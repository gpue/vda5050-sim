"""Order-conflict error type constants.

nova_vda5050.errors ships constants for connector-side faults (hardware,
communication, e-stop...) but not for the order/instant-action validation
outcomes a fleet-control-facing AGV needs to report. VDA5050's errorType is a
free-form string (not a closed enum), so these follow the same lowerCamelCase
convention as nova_vda5050.errors' own constants.
"""

from __future__ import annotations

OTHER_ORDER_ACTIVE = "otherOrderActive"
OUTDATED_ORDER_UPDATE = "outdatedOrderUpdate"
SAME_ORDER_UPDATE_ID = "sameOrderUpdateId"
NO_ORDER_TO_CANCEL = "noOrderToCancel"
VALIDATION_ERROR = "validationError"
