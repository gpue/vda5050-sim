"""Error-type constants and builder.

VDA5050's `errorType` is a free-form string (not a closed enum), so these
follow a lowerCamelCase convention consistent with common real-world
implementations.
"""

from __future__ import annotations

from vda5050_sim.schemas import Error, ErrorLevel, ErrorReference

# Order/instant-action validation outcomes.
OTHER_ORDER_ACTIVE = "otherOrderActive"
OUTDATED_ORDER_UPDATE = "outdatedOrderUpdate"
SAME_ORDER_UPDATE_ID = "sameOrderUpdateId"
NO_ORDER_TO_CANCEL = "noOrderToCancel"
VALIDATION_ERROR = "validationError"

# Connector-side faults.
HARDWARE_FAULT = "hardwareFault"

# Map/zone-set lifecycle (spec Section 6.3, 6.4).
UNKNOWN_MAP_ID = "unknownMapId"
DUPLICATE_MAP = "duplicateMap"
DUPLICATE_ZONE_SET = "duplicateZoneSet"

# Spec-defined: "Receival of an unsupported instant action" (errorLevel
# WARNING) — used both for genuinely unknown action types and for actions
# that exist in the spec but require a protocol version newer than what a
# legacy-simulated robot announces (see agv.py's LEGACY_UNSUPPORTED_ACTIONS).
INVALID_INSTANT_ACTION = "invalidInstantAction"


def make_error(
    error_type: str,
    description: str | None = None,
    level: ErrorLevel = ErrorLevel.WARNING,
    hint: str | None = None,
    references: dict[str, str] | None = None,
) -> Error:
    refs = [ErrorReference(referenceKey=k, referenceValue=v) for k, v in (references or {}).items()]
    return Error(errorType=error_type, errorLevel=level, errorDescription=description, errorHint=hint, errorReferences=refs)
