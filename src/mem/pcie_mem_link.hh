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

/**
 * @file
 * PCIeMemLink: a PCIe link in front of a memory device (a PCIe or CXL.mem
 * style memory expander). See PCIeMemLink.py for the model.
 */

#ifndef __MEM_PCIE_MEM_LINK_HH__
#define __MEM_PCIE_MEM_LINK_HH__

#include <deque>

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/PCIeMemLink.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PCIeMemLink : public SimObject
{
  public:
    PARAMS(PCIeMemLink);
    PCIeMemLink(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;
    void init() override;
    DrainState drain() override;

  private:
    class CpuSidePort : public ResponsePort
    {
      public:
        CpuSidePort(const std::string &name, PCIeMemLink &link)
            : ResponsePort(name), link(link)
        {}

      protected:
        bool recvTimingReq(PacketPtr pkt) override
        { return link.recvTimingReq(pkt); }
        void recvRespRetry() override { link.recvRespRetry(); }
        Tick recvAtomic(PacketPtr pkt) override
        { return link.recvAtomic(pkt); }
        Tick recvAtomicBackdoor(PacketPtr pkt,
                                MemBackdoorPtr &backdoor) override;
        void recvFunctional(PacketPtr pkt) override
        { link.recvFunctional(pkt); }
        void recvMemBackdoorReq(const MemBackdoorReq &req,
                                MemBackdoorPtr &backdoor) override;
        AddrRangeList getAddrRanges() const override;

      private:
        PCIeMemLink &link;
    };

    class MemSidePort : public RequestPort
    {
      public:
        MemSidePort(const std::string &name, PCIeMemLink &link)
            : RequestPort(name), link(link)
        {}

      protected:
        bool recvTimingResp(PacketPtr pkt) override
        { return link.recvTimingResp(pkt); }
        void recvReqRetry() override { link.recvReqRetry(); }
        void recvRangeChange() override
        { link.cpuSidePort.sendRangeChange(); }

      private:
        PCIeMemLink &link;
    };

    /** Remembers when a non-posted request entered the link. */
    struct LinkSenderState : public Packet::SenderState
    {
        explicit LinkSenderState(Tick t) : entered(t) {}
        Tick entered;
    };

    /** A packet waiting in one of the link's queues until `ready`. */
    struct Entry
    {
        PacketPtr pkt;
        Tick ready;
    };

    /** One direction of the link: a serializer at a fixed byte rate. */
    struct Direction
    {
        Tick busyUntil = 0;
    };

    /** TLP count, wire bytes and payload bytes of one transfer. */
    struct Wire
    {
        unsigned tlps;
        unsigned bytes;
        unsigned payload;
    };

    // Port handlers
    bool recvTimingReq(PacketPtr pkt);
    void recvRespRetry();
    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();

    // Pipeline stages, one event each
    void processDownstream();   ///< root port -> link -> device
    void processDevRequest();   ///< device -> memory controller
    void processUpstream();     ///< device -> link -> root port
    void processHostResponse(); ///< root port -> memory bus
    void processCreditReturn();

    void schedAt(EventFunctionWrapper &ev, Tick when);
    void trySendRetry();
    void checkDrain();
    bool idle() const;

    Wire requestWire(PacketPtr pkt) const;
    Wire completionWire(PacketPtr pkt) const;
    Tick serialize(unsigned bytes) const;
    Tick unloadedLatency(PacketPtr pkt) const;

    CpuSidePort cpuSidePort;
    MemSidePort memSidePort;

    // Configuration
    const double ticksPerByte;
    const unsigned tlpOverhead;
    const unsigned maxPayload;
    const unsigned maxReadReq;
    const Tick rcLatency;
    const Tick phyLatency;
    const Tick devLatency;
    const unsigned maxTags;
    const unsigned hostQueueLimit;

    // State
    std::deque<Entry> hostReqQ;  ///< at the root port, waiting for the link
    std::deque<Entry> devReqQ;   ///< at the device, waiting for memory
    std::deque<Entry> devRespQ;  ///< at the device, waiting for the link
    std::deque<Entry> hostRespQ; ///< at the root port, waiting for the bus
    std::deque<Tick> creditReturns; ///< when each in-flight credit is back

    Direction down;
    Direction up;
    unsigned credits;
    unsigned tagsInUse = 0;
    bool retryReq = false;       ///< the bus is owed a retry
    bool waitMemRetry = false;   ///< memory controller refused a request
    bool waitRespRetry = false;  ///< the bus refused a response
    Tick creditStallSince = MaxTick;

    EventFunctionWrapper downEvent;
    EventFunctionWrapper devReqEvent;
    EventFunctionWrapper upEvent;
    EventFunctionWrapper hostRespEvent;
    EventFunctionWrapper creditEvent;

    struct LinkStats : public statistics::Group
    {
        LinkStats(PCIeMemLink &link);

        statistics::Scalar readReqs;
        statistics::Scalar writeReqs;
        statistics::Scalar postedWrites;
        statistics::Scalar downTLPs;
        statistics::Scalar upTLPs;
        statistics::Scalar downWireBytes;
        statistics::Scalar upWireBytes;
        statistics::Scalar downPayloadBytes;
        statistics::Scalar upPayloadBytes;
        statistics::Scalar downBusyTicks;
        statistics::Scalar upBusyTicks;
        statistics::Scalar creditStallTicks;
        statistics::Scalar hostQueueFullRetries;
        statistics::Scalar tagLimitRetries;
        statistics::Scalar respondedReqs;
        statistics::Scalar totRespLatency;
        statistics::Histogram respLatency;

        statistics::Formula downUtil;
        statistics::Formula upUtil;
        statistics::Formula avgRespLatency;
    } stats;
};

} // namespace gem5

#endif // __MEM_PCIE_MEM_LINK_HH__
