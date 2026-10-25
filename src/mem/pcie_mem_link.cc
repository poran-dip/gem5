/*
 * Copyright (c) 2026 The tiered-memory authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/pcie_mem_link.hh"

#include <algorithm>
#include <cmath>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PCIeMemLink.hh"
#include "sim/core.hh"
#include "sim/stats.hh"

namespace gem5
{

namespace
{

/** TLP header bytes: 3 DW below 4 GiB, 4 DW (64-bit address) above. */
unsigned
requestHeaderBytes(Addr addr)
{
    return addr >= (Addr(1) << 32) ? 16 : 12;
}

/** Completion (Cpl/CplD) header: always 3 DW. */
constexpr unsigned CompletionHeaderBytes = 12;

/** Ticks per byte on one direction of the link. */
double
linkTicksPerByte(const PCIeMemLinkParams &p)
{
    // Transfer rate per lane (GT/s) and line-code efficiency per generation.
    static const double gts[] = {0, 2.5, 5.0, 8.0, 16.0, 32.0};
    fatal_if(p.gen < 1 || p.gen > 5,
             "%s: PCIe gen must be 1 to 5, got %d", p.name, p.gen);
    fatal_if(p.lanes == 0 || p.lanes > 32 || !isPowerOf2(p.lanes),
             "%s: lanes must be 1, 2, 4, 8, 16 or 32, got %d",
             p.name, p.lanes);
    fatal_if(p.dll_overhead < 0 || p.dll_overhead >= 1,
             "%s: dll_overhead must be in [0, 1), got %f",
             p.name, p.dll_overhead);
    const double encoding = p.gen <= 2 ? 8.0 / 10.0 : 128.0 / 130.0;
    const double bytes_per_sec = p.lanes * gts[p.gen] * 1e9 * encoding / 8.0
                                 * (1.0 - p.dll_overhead);
    return sim_clock::as_float::s / bytes_per_sec;
}

} // anonymous namespace

PCIeMemLink::PCIeMemLink(const Params &p)
    : SimObject(p),
      cpuSidePort(p.name + ".cpu_side_port", *this),
      memSidePort(p.name + ".mem_side_port", *this),
      ticksPerByte(linkTicksPerByte(p)),
      tlpOverhead(p.tlp_overhead + (p.ecrc ? 4 : 0)),
      maxPayload(p.max_payload),
      maxReadReq(p.max_read_req),
      rcLatency(p.rc_latency),
      phyLatency(p.phy_latency),
      devLatency(p.dev_latency),
      maxTags(p.tags),
      hostQueueLimit(p.host_queue),
      credits(p.dev_credits),
      downEvent([this]{ processDownstream(); }, name() + ".down"),
      devReqEvent([this]{ processDevRequest(); }, name() + ".devReq"),
      upEvent([this]{ processUpstream(); }, name() + ".up"),
      hostRespEvent([this]{ processHostResponse(); }, name() + ".hostResp"),
      creditEvent([this]{ processCreditReturn(); }, name() + ".credit"),
      stats(*this)
{
    fatal_if(maxPayload == 0 || maxReadReq == 0,
             "%s: max_payload and max_read_req must be positive", name());
    fatal_if(maxTags == 0 || p.dev_credits == 0 || hostQueueLimit == 0,
             "%s: tags, dev_credits and host_queue must be positive",
             name());
}

Port &
PCIeMemLink::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_side_port")
        return cpuSidePort;
    if (if_name == "mem_side_port")
        return memSidePort;
    return SimObject::getPort(if_name, idx);
}

void
PCIeMemLink::init()
{
    fatal_if(!cpuSidePort.isConnected() || !memSidePort.isConnected(),
             "%s: both ports must be connected", name());
    cpuSidePort.sendRangeChange();
}

// ---------------------------------------------------------------------------
// Wire accounting

PCIeMemLink::Wire
PCIeMemLink::requestWire(PacketPtr pkt) const
{
    const unsigned size = pkt->getSize();
    const unsigned hdr = requestHeaderBytes(pkt->getAddr()) + tlpOverhead;
    Wire w{1, 0, 0};
    if (pkt->hasData()) {
        // MWr: the data travels with the request, split at MPS.
        w.tlps = std::max(1u, divCeil(size, maxPayload));
        w.payload = size;
    } else if (pkt->isRead()) {
        // MRd: header only, split at MRRS.
        w.tlps = std::max(1u, divCeil(size, maxReadReq));
    }
    w.bytes = w.tlps * hdr + w.payload;
    return w;
}

PCIeMemLink::Wire
PCIeMemLink::completionWire(PacketPtr pkt) const
{
    const unsigned hdr = CompletionHeaderBytes + tlpOverhead;
    Wire w{1, 0, 0};
    if (pkt->hasData()) {
        // CplD, split at MPS.
        w.payload = pkt->getSize();
        w.tlps = std::max(1u, divCeil(w.payload, maxPayload));
    }
    w.bytes = w.tlps * hdr + w.payload;
    return w;
}

Tick
PCIeMemLink::serialize(unsigned bytes) const
{
    return static_cast<Tick>(std::ceil(bytes * ticksPerByte));
}

Tick
PCIeMemLink::unloadedLatency(PacketPtr pkt) const
{
    // Used for atomic accesses (boot and fast-forward only): the trip with
    // an idle link, and nothing queued.
    Tick t = rcLatency + serialize(requestWire(pkt).bytes) + phyLatency +
             devLatency;
    if (pkt->needsResponse()) {
        const unsigned data = pkt->isRead() ? pkt->getSize() : 0;
        const unsigned tlps = data ? divCeil(data, maxPayload) : 1;
        t += devLatency +
             serialize(tlps * (CompletionHeaderBytes + tlpOverhead) + data) +
             phyLatency + rcLatency;
    }
    return t;
}

// ---------------------------------------------------------------------------
// Event helpers

void
PCIeMemLink::schedAt(EventFunctionWrapper &ev, Tick when)
{
    when = std::max(when, curTick());
    if (!ev.scheduled())
        schedule(ev, when);
    else if (ev.when() > when)
        reschedule(ev, when);
}

void
PCIeMemLink::trySendRetry()
{
    if (retryReq && hostReqQ.size() < hostQueueLimit &&
        tagsInUse < maxTags) {
        DPRINTF(PCIeMemLink, "sending retry to the bus\n");
        retryReq = false;
        cpuSidePort.sendRetryReq();
    }
}

bool
PCIeMemLink::idle() const
{
    return hostReqQ.empty() && devReqQ.empty() && devRespQ.empty() &&
           hostRespQ.empty() && tagsInUse == 0;
}

void
PCIeMemLink::checkDrain()
{
    if (drainState() == DrainState::Draining && idle()) {
        DPRINTF(PCIeMemLink, "drained\n");
        signalDrainDone();
    }
}

DrainState
PCIeMemLink::drain()
{
    return idle() ? DrainState::Drained : DrainState::Draining;
}

// ---------------------------------------------------------------------------
// Request path: bus -> root port -> link -> device -> memory controller

bool
PCIeMemLink::recvTimingReq(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(),
             "%s: should not see packets where a cache is responding",
             name());

    // A retry is owed for an earlier refusal; keep refusing until it is
    // sent so requests are accepted in order.
    if (retryReq)
        return false;

    if (hostReqQ.size() >= hostQueueLimit) {
        ++stats.hostQueueFullRetries;
        retryReq = true;
        return false;
    }
    const bool non_posted = pkt->needsResponse();
    if (non_posted && tagsInUse >= maxTags) {
        ++stats.tagLimitRetries;
        retryReq = true;
        return false;
    }

    if (non_posted) {
        ++tagsInUse;
        pkt->pushSenderState(new LinkSenderState(curTick()));
    }
    if (pkt->isRead()) {
        ++stats.readReqs;
    } else if (pkt->isWrite()) {
        ++stats.writeReqs;
        if (!non_posted)
            ++stats.postedWrites;
    }

    // The packet reaches us after the bus's header and payload delays.
    const Tick ready = curTick() + pkt->headerDelay + pkt->payloadDelay +
                       rcLatency;
    pkt->headerDelay = pkt->payloadDelay = 0;
    DPRINTF(PCIeMemLink, "req %s addr %#x ready at %d\n",
            pkt->cmdString(), pkt->getAddr(), ready);
    hostReqQ.push_back({pkt, ready});
    schedAt(downEvent, std::max(hostReqQ.front().ready, down.busyUntil));
    return true;
}

void
PCIeMemLink::processDownstream()
{
    if (hostReqQ.empty())
        return;
    const Tick now = curTick();
    const Entry head = hostReqQ.front();
    if (head.ready > now || down.busyUntil > now) {
        schedAt(downEvent, std::max(head.ready, down.busyUntil));
        return;
    }
    if (credits == 0) {
        // Out of device buffer credits: wait for processCreditReturn.
        if (creditStallSince == MaxTick)
            creditStallSince = now;
        return;
    }
    if (creditStallSince != MaxTick) {
        stats.creditStallTicks += now - creditStallSince;
        creditStallSince = MaxTick;
    }

    hostReqQ.pop_front();
    --credits;
    const Wire w = requestWire(head.pkt);
    const Tick ser = serialize(w.bytes);
    down.busyUntil = now + ser;
    stats.downTLPs += w.tlps;
    stats.downWireBytes += w.bytes;
    stats.downPayloadBytes += w.payload;
    stats.downBusyTicks += ser;

    devReqQ.push_back({head.pkt, now + ser + phyLatency + devLatency});
    schedAt(devReqEvent, devReqQ.front().ready);
    if (!hostReqQ.empty())
        schedAt(downEvent, std::max(hostReqQ.front().ready, down.busyUntil));
    trySendRetry();
}

void
PCIeMemLink::processDevRequest()
{
    if (devReqQ.empty() || waitMemRetry)
        return;
    const Entry head = devReqQ.front();
    if (head.ready > curTick()) {
        schedAt(devReqEvent, head.ready);
        return;
    }
    devReqQ.pop_front();
    if (!memSidePort.sendTimingReq(head.pkt)) {
        devReqQ.push_front(head);
        waitMemRetry = true;
        return;
    }
    // The buffer entry is free again; the credit reaches the host one
    // link traversal later (an UpdateFC DLLP).
    creditReturns.push_back(curTick() + phyLatency);
    schedAt(creditEvent, creditReturns.front());
    if (!devReqQ.empty())
        schedAt(devReqEvent, devReqQ.front().ready);
    checkDrain();
}

void
PCIeMemLink::recvReqRetry()
{
    waitMemRetry = false;
    processDevRequest();
}

void
PCIeMemLink::processCreditReturn()
{
    const Tick now = curTick();
    while (!creditReturns.empty() && creditReturns.front() <= now) {
        creditReturns.pop_front();
        ++credits;
    }
    if (!creditReturns.empty())
        schedAt(creditEvent, creditReturns.front());
    if (!hostReqQ.empty())
        schedAt(downEvent, std::max(hostReqQ.front().ready, down.busyUntil));
}

// ---------------------------------------------------------------------------
// Completion path: memory controller -> device -> link -> root port -> bus

bool
PCIeMemLink::recvTimingResp(PacketPtr pkt)
{
    // Space was reserved by the tag taken when the request was accepted.
    const Tick ready = curTick() + pkt->headerDelay + pkt->payloadDelay +
                       devLatency;
    pkt->headerDelay = pkt->payloadDelay = 0;
    devRespQ.push_back({pkt, ready});
    schedAt(upEvent, std::max(devRespQ.front().ready, up.busyUntil));
    return true;
}

void
PCIeMemLink::processUpstream()
{
    if (devRespQ.empty())
        return;
    const Tick now = curTick();
    const Entry head = devRespQ.front();
    if (head.ready > now || up.busyUntil > now) {
        schedAt(upEvent, std::max(head.ready, up.busyUntil));
        return;
    }
    // Root ports advertise infinite completion credits: no check here.
    devRespQ.pop_front();
    const Wire w = completionWire(head.pkt);
    const Tick ser = serialize(w.bytes);
    up.busyUntil = now + ser;
    stats.upTLPs += w.tlps;
    stats.upWireBytes += w.bytes;
    stats.upPayloadBytes += w.payload;
    stats.upBusyTicks += ser;

    hostRespQ.push_back({head.pkt, now + ser + phyLatency + rcLatency});
    schedAt(hostRespEvent, hostRespQ.front().ready);
    if (!devRespQ.empty())
        schedAt(upEvent, std::max(devRespQ.front().ready, up.busyUntil));
}

void
PCIeMemLink::processHostResponse()
{
    if (hostRespQ.empty() || waitRespRetry)
        return;
    const Tick now = curTick();
    const Entry head = hostRespQ.front();
    if (head.ready > now) {
        schedAt(hostRespEvent, head.ready);
        return;
    }
    PacketPtr pkt = head.pkt;
    auto *st = dynamic_cast<LinkSenderState *>(pkt->senderState);
    panic_if(!st, "%s: response without its link sender state", name());
    pkt->popSenderState();
    hostRespQ.pop_front();
    if (!cpuSidePort.sendTimingResp(pkt)) {
        pkt->pushSenderState(st);
        hostRespQ.push_front(head);
        waitRespRetry = true;
        return;
    }
    const Tick lat = now - st->entered;
    delete st;
    ++stats.respondedReqs;
    stats.totRespLatency += lat;
    stats.respLatency.sample(lat);
    --tagsInUse;

    if (!hostRespQ.empty())
        schedAt(hostRespEvent, hostRespQ.front().ready);
    trySendRetry();
    checkDrain();
}

void
PCIeMemLink::recvRespRetry()
{
    waitRespRetry = false;
    processHostResponse();
}

// ---------------------------------------------------------------------------
// Atomic and functional accesses

Tick
PCIeMemLink::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(),
             "%s: should not see packets where a cache is responding",
             name());
    const Tick lat = unloadedLatency(pkt);
    return lat + memSidePort.sendAtomic(pkt);
}

Tick
PCIeMemLink::CpuSidePort::recvAtomicBackdoor(PacketPtr pkt,
                                             MemBackdoorPtr &backdoor)
{
    const Tick lat = link.unloadedLatency(pkt);
    return lat + link.memSidePort.sendAtomicBackdoor(pkt, backdoor);
}

void
PCIeMemLink::CpuSidePort::recvMemBackdoorReq(const MemBackdoorReq &req,
                                             MemBackdoorPtr &backdoor)
{
    link.memSidePort.sendMemBackdoorReq(req, backdoor);
}

AddrRangeList
PCIeMemLink::CpuSidePort::getAddrRanges() const
{
    return link.memSidePort.getAddrRanges();
}

void
PCIeMemLink::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());
    // Newest data first: requests still on the host side, then those at
    // the device, then completions on their way back.
    for (auto *q : {&hostReqQ, &devReqQ, &devRespQ, &hostRespQ}) {
        for (const auto &e : *q) {
            if (pkt->trySatisfyFunctional(e.pkt)) {
                pkt->popLabel();
                pkt->makeResponse();
                return;
            }
        }
    }
    pkt->popLabel();
    memSidePort.sendFunctional(pkt);
}

// ---------------------------------------------------------------------------
// Statistics

PCIeMemLink::LinkStats::LinkStats(PCIeMemLink &link)
    : statistics::Group(&link),
      ADD_STAT(readReqs, statistics::units::Count::get(),
               "Read requests that entered the link"),
      ADD_STAT(writeReqs, statistics::units::Count::get(),
               "Write requests that entered the link"),
      ADD_STAT(postedWrites, statistics::units::Count::get(),
               "Writes that needed no response (posted, e.g. writebacks)"),
      ADD_STAT(downTLPs, statistics::units::Count::get(),
               "TLPs sent host to device"),
      ADD_STAT(upTLPs, statistics::units::Count::get(),
               "TLPs sent device to host"),
      ADD_STAT(downWireBytes, statistics::units::Byte::get(),
               "Bytes sent host to device: headers, framing and payload"),
      ADD_STAT(upWireBytes, statistics::units::Byte::get(),
               "Bytes sent device to host: headers, framing and payload"),
      ADD_STAT(downPayloadBytes, statistics::units::Byte::get(),
               "Payload (data) bytes sent host to device"),
      ADD_STAT(upPayloadBytes, statistics::units::Byte::get(),
               "Payload (data) bytes sent device to host"),
      ADD_STAT(downBusyTicks, statistics::units::Tick::get(),
               "Ticks the host-to-device direction was transmitting"),
      ADD_STAT(upBusyTicks, statistics::units::Tick::get(),
               "Ticks the device-to-host direction was transmitting"),
      ADD_STAT(creditStallTicks, statistics::units::Tick::get(),
               "Ticks a ready request waited for a device credit"),
      ADD_STAT(hostQueueFullRetries, statistics::units::Count::get(),
               "Requests refused because the root port queue was full"),
      ADD_STAT(tagLimitRetries, statistics::units::Count::get(),
               "Requests refused because all tags were in use"),
      ADD_STAT(respondedReqs, statistics::units::Count::get(),
               "Non-posted requests completed"),
      ADD_STAT(totRespLatency, statistics::units::Tick::get(),
               "Sum of round-trip latencies of completed requests, from "
               "entering the link to the response leaving it"),
      ADD_STAT(respLatency, statistics::units::Tick::get(),
               "Round-trip latency of completed requests"),
      ADD_STAT(downUtil, statistics::units::Ratio::get(),
               "Host-to-device link utilization"),
      ADD_STAT(upUtil, statistics::units::Ratio::get(),
               "Device-to-host link utilization"),
      ADD_STAT(avgRespLatency, statistics::units::Rate<
                   statistics::units::Tick, statistics::units::Count>::get(),
               "Average round-trip latency of completed requests")
{
    respLatency.init(20);
    downUtil = downBusyTicks / simTicks;
    upUtil = upBusyTicks / simTicks;
    avgRespLatency = totRespLatency / respondedReqs;
}

} // namespace gem5
