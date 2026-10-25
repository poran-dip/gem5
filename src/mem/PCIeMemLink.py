# Copyright (c) 2026 The tiered-memory authors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.SimObject import SimObject


class PCIeMemLink(SimObject):
    """A PCIe link between the host and a memory device behind it.

    Sits between the host's memory bus (cpu_side_port) and the device's
    memory controller (mem_side_port), so the memory behind it stays
    ordinary cacheable system memory (the way a CXL.mem or PCIe memory
    expander appears to the OS), while every access pays for the trip:

      root complex -> downstream link -> device controller -> DRAM
      DRAM -> device controller -> upstream link -> root complex

    Each direction of the link is a serializer whose rate comes from the
    generation and lane count (with line encoding and a data-link-layer
    overhead fraction); every request and completion is sent as one or more
    TLPs with their header and framing bytes. Non-posted requests (reads)
    need a tag; requests need a flow-control credit at the device, returned
    when the device hands the request to its memory controller.

    Defaults are a PCIe Gen4 x8 link with plain-PCIe latencies; see the
    tiered-memory repo (docs/pcie-link.md) for the sources.
    """

    type = "PCIeMemLink"
    cxx_header = "mem/pcie_mem_link.hh"
    cxx_class = "gem5::PCIeMemLink"

    cpu_side_port = ResponsePort(
        "Host side: receives requests from the memory bus, sends responses"
    )
    mem_side_port = RequestPort(
        "Device side: sends requests to the device's memory controller"
    )

    # Physical layer
    gen = Param.Unsigned(4, "PCIe generation, 1 to 5")
    lanes = Param.Unsigned(8, "Link width in lanes: 1, 2, 4, 8, 16 or 32")
    dll_overhead = Param.Float(
        0.08,
        "Fraction of link bandwidth taken by data link layer traffic "
        "(ACK/NAK and flow-control update DLLPs) and ordered sets",
    )

    # Transaction layer
    tlp_overhead = Param.Unsigned(
        8,
        "Bytes per TLP besides header and payload: physical-layer framing "
        "(2), sequence number (2) and LCRC (4)",
    )
    ecrc = Param.Bool(False, "Append a 4-byte end-to-end CRC to every TLP")
    max_payload = Param.Unsigned(256, "Max Payload Size (MPS), bytes")
    max_read_req = Param.Unsigned(512, "Max Read Request Size (MRRS), bytes")

    # Latencies
    rc_latency = Param.Latency(
        "100ns",
        "Root complex / root port processing, paid once by each request on "
        "its way out and once by each completion on its way back",
    )
    phy_latency = Param.Latency(
        "30ns",
        "One-way PHY (serdes, both ends) and wire latency, per traversal",
    )
    dev_latency = Param.Latency(
        "80ns",
        "Device controller processing, paid once by each request on its way "
        "in and once by each completion on its way out",
    )

    # Concurrency limits
    tags = Param.Unsigned(
        256, "Non-posted requests (reads) the host can have outstanding"
    )
    dev_credits = Param.Unsigned(
        64,
        "Request buffer entries at the device (flow-control header "
        "credits); a credit returns when the device forwards the request to "
        "its memory controller",
    )
    host_queue = Param.Unsigned(
        128, "Requests the root port can queue before back-pressuring the bus"
    )
