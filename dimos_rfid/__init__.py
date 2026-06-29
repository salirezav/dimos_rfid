"""DimOS integration for the Vulcan RFID scanner."""

from dimos_rfid.msgs import RfidTag, RfidTagArray
from dimos_rfid.rfid_module import RfidModule, RfidModuleConfig

__all__ = [
    "RfidModule",
    "RfidModuleConfig",
    "RfidTag",
    "RfidTagArray",
]
