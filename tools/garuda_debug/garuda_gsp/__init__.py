"""
garuda_gsp — shared Garuda Serial Protocol library.

One protocol implementation for every Garuda debug tool. Version-negotiated
(handles INFO 20/24B, snapshot 68..228B) so it survives firmware wire changes.

    from garuda_gsp import GspClient, Session
    with GspClient() as c:        # auto-detects the port
        print(c.get_info())
"""
from . import protocol
from .framing import crc16, build_packet, read_packet, send_cmd
from .decode import decode_info, decode_snapshot
from .client import GspClient, GspError, find_port, list_ports_human
from .session import Session

__version__ = "0.1.0"

__all__ = [
    "protocol", "crc16", "build_packet", "read_packet", "send_cmd",
    "decode_info", "decode_snapshot", "GspClient", "GspError",
    "find_port", "list_ports_human", "Session", "__version__",
]
