"""DimOS integration for the Vulcan RFID scanner."""

from dimos_rfid.msgs import RfidTag, RfidTagArray
from dimos_rfid.rfid_module import RfidModule, RfidModuleConfig
from dimos_rfid.rfid_semantic_localizer import (
    RfidSemanticLocalizerConfig,
    RfidSemanticLocalizerModule,
)
from dimos_rfid.rfid_tracker import RFIDTracker
from dimos_rfid.semantic_map import MaterialClass, SemanticOccupancyGrid3D
from dimos_rfid.semantic_particle_filter import SemanticParticleFilter3D

__all__ = [
    "MaterialClass",
    "RFIDTracker",
    "RfidModule",
    "RfidModuleConfig",
    "RfidSemanticLocalizerConfig",
    "RfidSemanticLocalizerModule",
    "RfidTag",
    "RfidTagArray",
    "SemanticOccupancyGrid3D",
    "SemanticParticleFilter3D",
]
