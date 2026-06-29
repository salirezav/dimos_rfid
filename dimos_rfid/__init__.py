"""DimOS integration for the Vulcan RFID scanner."""

from dimos_rfid.msgs import RfidTag, RfidTagArray
from dimos_rfid.rfid_module import RfidModule, RfidModuleConfig
from dimos_rfid.rfid_overlay_module import RfidCameraOverlay, RfidOverlayModule

__all__ = [
    "RfidCameraOverlay",
    "RfidModule",
    "RfidModuleConfig",
    "RfidOverlayModule",
    "RfidTag",
    "RfidTagArray",
]
