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


def make_error(
    error_type: str,
    description: str | None = None,
    level: ErrorLevel = ErrorLevel.WARNING,
    hint: str | None = None,
    references: dict[str, str] | None = None,
) -> Error:
    refs = [ErrorReference(referenceKey=k, referenceValue=v) for k, v in (references or {}).items()]
    return Error(errorType=error_type, errorLevel=level, errorDescription=description, errorHint=hint, errorReferences=refs)
