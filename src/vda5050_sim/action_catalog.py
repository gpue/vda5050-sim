"""The VDA5050 v3.0.0 "Predefined Actions" vocabulary (spec Section 6.2.3,
Table 4), transcribed directly from the official spec text
(github.com/VDA5050/VDA5050, VDA5050_EN.md) rather than from memory — every
actionType name, its instant/node/edge/zone scope, and its parameter list are
taken verbatim from that table.

Two upstream spec inconsistencies, applied with the same "obvious typo, not a
meaningful spec choice" treatment as the JSON-schema fixes documented in
`json_schemas/README.md`:
- `startHibernation`/`stopHibernation`'s Table 4 rows are one column short in
  the raw source (9 pipe-delimited fields vs. the header's 10) — the trailing
  `zone` column is simply missing. Treated as `zone: False`, consistent with
  every other value in those rows and with the fact that a connection-state
  control action has no sensible zone scope.

`blockingTypes`/`pauseAllowed`/`cancelAllowed` are NOT spec-mandated per
actionType — VDA5050 leaves these as each manufacturer's own declared
capability. The values below are this simulator's own reasonable choices,
grounded where possible in Table 5's per-action state-transition semantics
(e.g. `pauseAllowed=True` only for the four actions whose PAUSED column in
Table 5 actually describes real behavior: pick, drop, finePositioning, and —
via the general order/instant-action pause mechanism — nothing else).

`core=True` actions are always available on every robot (they're
protocol-mandatory or protocol-machinery). `core=False` actions are
capability-gated by `RobotConfig.supported_actions`, same as the pre-existing
generic-instant-action passthrough.
"""

from __future__ import annotations

from dataclasses import dataclass

from vda5050_sim.schemas import ActionScope, BlockingType


@dataclass(frozen=True)
class ActionParamDef:
    key: str
    value_data_type: str
    description: str = ""
    optional: bool = True


@dataclass(frozen=True)
class ActionDef:
    action_type: str
    description: str
    scopes: tuple[ActionScope, ...]
    blocking_types: tuple[BlockingType, ...] = (BlockingType.NONE,)
    pause_allowed: bool = False
    cancel_allowed: bool = True
    params: tuple[ActionParamDef, ...] = ()
    core: bool = True


_TIMER_BLOCKING = (BlockingType.NONE, BlockingType.SOFT, BlockingType.SINGLE, BlockingType.HARD)
_CONTROL_BLOCKING = (BlockingType.NONE,)


ACTION_CATALOG: dict[str, ActionDef] = {
    "startPause": ActionDef(
        "startPause", "Activates the pause mode.", (ActionScope.INSTANT,), _CONTROL_BLOCKING
    ),
    "stopPause": ActionDef(
        "stopPause", "Deactivates the pause mode.", (ActionScope.INSTANT,), _CONTROL_BLOCKING
    ),
    "startHibernation": ActionDef(
        "startHibernation",
        "Initiates hibernate mode: robot stays connected but stops sending state; publishes connectionState HIBERNATING.",
        (ActionScope.INSTANT,),
        _CONTROL_BLOCKING,
        cancel_allowed=False,
        params=(ActionParamDef("wakeUpTime", "string", "ISO8601 timestamp to auto stopHibernation", optional=True),),
    ),
    "stopHibernation": ActionDef(
        "stopHibernation",
        "Ends hibernate mode; publishes connectionState ONLINE.",
        (ActionScope.INSTANT,),
        _CONTROL_BLOCKING,
        cancel_allowed=False,
    ),
    "shutdown": ActionDef(
        "shutdown",
        "Coordinated shutdown; requires the robot to be idle; publishes connectionState OFFLINE.",
        (ActionScope.INSTANT,),
        _CONTROL_BLOCKING,
        cancel_allowed=False,
    ),
    "startCharging": ActionDef(
        "startCharging", "Activates the charging process.", (ActionScope.INSTANT, ActionScope.NODE), _TIMER_BLOCKING
    ),
    "stopCharging": ActionDef(
        "stopCharging", "Discontinues the charging process.", (ActionScope.INSTANT, ActionScope.NODE), _TIMER_BLOCKING
    ),
    "initializePosition": ActionDef(
        "initializePosition",
        "Resets (overrides) the pose of the mobile robot with the given parameters.",
        (ActionScope.INSTANT, ActionScope.NODE),
        _TIMER_BLOCKING,
        params=(
            ActionParamDef("x", "float64"),
            ActionParamDef("y", "float64"),
            ActionParamDef("theta", "float64"),
            ActionParamDef("mapId", "string"),
            ActionParamDef("lastNodeId", "string"),
        ),
    ),
    "enableMap": ActionDef(
        "enableMap",
        "Enables a previously downloaded map for use without initializing a new position.",
        (ActionScope.INSTANT, ActionScope.NODE),
        _TIMER_BLOCKING,
        params=(ActionParamDef("mapId", "string"), ActionParamDef("mapVersion", "string")),
    ),
    "downloadMap": ActionDef(
        "downloadMap",
        "Triggers the download of a new map.",
        (ActionScope.INSTANT,),
        _TIMER_BLOCKING,
        cancel_allowed=False,
        params=(
            ActionParamDef("mapId", "string"),
            ActionParamDef("mapVersion", "string"),
            ActionParamDef("mapDownloadLink", "string"),
            ActionParamDef("mapHash", "string", optional=True),
        ),
    ),
    "deleteMap": ActionDef(
        "deleteMap",
        "Removes a map from the mobile robot's memory.",
        (ActionScope.INSTANT,),
        _TIMER_BLOCKING,
        params=(ActionParamDef("mapId", "string"), ActionParamDef("mapVersion", "string")),
    ),
    "downloadZoneSet": ActionDef(
        "downloadZoneSet",
        "Triggers the download of a zone set.",
        (ActionScope.INSTANT,),
        _TIMER_BLOCKING,
        cancel_allowed=False,
        params=(
            ActionParamDef("zoneSetId", "string"),
            ActionParamDef("zoneSetDownloadLink", "string"),
            ActionParamDef("zoneSetHash", "string", optional=True),
        ),
    ),
    "enableZoneSet": ActionDef(
        "enableZoneSet",
        "Enables a previously downloaded zone set for use in orders.",
        (ActionScope.INSTANT, ActionScope.NODE),
        _TIMER_BLOCKING,
        params=(ActionParamDef("zoneSetId", "string"),),
    ),
    "deleteZoneSet": ActionDef(
        "deleteZoneSet",
        "Removes a zone set from the mobile robot's memory.",
        (ActionScope.INSTANT,),
        _TIMER_BLOCKING,
        params=(ActionParamDef("zoneSetId", "string"),),
    ),
    "clearInstantActions": ActionDef(
        "clearInstantActions",
        "Removes all FINISHED or FAILED instant actions from the mobile robot state.",
        (ActionScope.INSTANT, ActionScope.NODE),
        _CONTROL_BLOCKING,
    ),
    "clearZoneActions": ActionDef(
        "clearZoneActions",
        "Removes all FINISHED or FAILED zone actions from the mobile robot's state.",
        (ActionScope.INSTANT, ActionScope.NODE),
        _CONTROL_BLOCKING,
    ),
    "stateRequest": ActionDef(
        "stateRequest", "Requests the mobile robot to send a new state message.", (ActionScope.INSTANT,), _CONTROL_BLOCKING
    ),
    "logReport": ActionDef(
        "logReport",
        "Requests the mobile robot to generate and store a log report.",
        (ActionScope.INSTANT,),
        _CONTROL_BLOCKING,
        params=(ActionParamDef("reason", "string"),),
    ),
    "pick": ActionDef(
        "pick",
        "Request the mobile robot to pick a load.",
        (ActionScope.NODE, ActionScope.EDGE),
        _TIMER_BLOCKING,
        pause_allowed=True,
        core=False,
        params=(
            ActionParamDef("lhd", "string", optional=True),
            ActionParamDef("stationType", "string", optional=True),
            ActionParamDef("stationName", "string", optional=True),
            ActionParamDef("loadType", "string", optional=True),
            ActionParamDef("loadId", "string", optional=True),
            ActionParamDef("height", "float64", optional=True),
            ActionParamDef("depth", "float64", optional=True),
            ActionParamDef("side", "string", optional=True),
        ),
    ),
    "drop": ActionDef(
        "drop",
        "Request the mobile robot to drop a load.",
        (ActionScope.NODE, ActionScope.EDGE),
        _TIMER_BLOCKING,
        pause_allowed=True,
        core=False,
        params=(
            ActionParamDef("lhd", "string", optional=True),
            ActionParamDef("stationType", "string", optional=True),
            ActionParamDef("stationName", "string", optional=True),
            ActionParamDef("loadType", "string", optional=True),
            ActionParamDef("loadId", "string", optional=True),
            ActionParamDef("height", "float64", optional=True),
            ActionParamDef("depth", "float64", optional=True),
        ),
    ),
    "detectObject": ActionDef(
        "detectObject",
        "Mobile robot detects an object (e.g., load, charging spot, free parking position).",
        (ActionScope.NODE, ActionScope.EDGE, ActionScope.ZONE),
        _TIMER_BLOCKING,
        core=False,
        params=(ActionParamDef("objectType", "string", optional=True),),
    ),
    "finePositioning": ActionDef(
        "finePositioning",
        "Mobile robot positions itself exactly on a target.",
        (ActionScope.NODE, ActionScope.EDGE, ActionScope.ZONE),
        _TIMER_BLOCKING,
        pause_allowed=True,
        core=False,
        params=(
            ActionParamDef("stationType", "string", optional=True),
            ActionParamDef("stationName", "string", optional=True),
        ),
    ),
    "waitForTrigger": ActionDef(
        "waitForTrigger",
        "Mobile robot waits for a trigger of the type(s) given in triggerType.",
        (ActionScope.NODE, ActionScope.ZONE),
        _TIMER_BLOCKING,
        core=False,
        params=(ActionParamDef("triggerType", "string[]"),),
    ),
    "trigger": ActionDef(
        "trigger",
        "Fleet control notifies the mobile robot that a waitForTrigger action has been released.",
        (ActionScope.INSTANT,),
        _CONTROL_BLOCKING,
    ),
    "retry": ActionDef(
        "retry",
        "Mobile robot retries the action given by actionId, currently in state RETRIABLE.",
        (ActionScope.INSTANT,),
        _CONTROL_BLOCKING,
        params=(ActionParamDef("actionId", "string"),),
    ),
    "skipRetry": ActionDef(
        "skipRetry",
        "Mobile robot skips the action given by actionId (currently RETRIABLE), setting it to FAILED.",
        (ActionScope.INSTANT,),
        _CONTROL_BLOCKING,
        params=(ActionParamDef("actionId", "string"),),
    ),
    "cancelOrder": ActionDef(
        "cancelOrder", "Mobile robot stops as soon as possible.", (ActionScope.INSTANT,), _CONTROL_BLOCKING,
        params=(ActionParamDef("orderId", "string", optional=True),),
    ),
    "factsheetRequest": ActionDef(
        "factsheetRequest", "Requests the mobile robot to send a factsheet.", (ActionScope.INSTANT,), _CONTROL_BLOCKING
    ),
    "updateCertificate": ActionDef(
        "updateCertificate",
        "Requests the mobile robot to download and activate a new certificate set (simulated, no real TLS).",
        (ActionScope.INSTANT,),
        _TIMER_BLOCKING,
        cancel_allowed=False,
        params=(
            ActionParamDef("service", "string"),
            ActionParamDef("keyDownloadLink", "string"),
            ActionParamDef("certificateDownloadLink", "string"),
            ActionParamDef("certificateAuthorityDownloadLink", "string", optional=True),
        ),
    ),
}


def scopes_for(action_type: str) -> tuple[ActionScope, ...] | None:
    """Returns None for actionTypes outside the catalog (manufacturer-custom
    actions declared via RobotConfig.supported_actions) — those have no
    Table-4 scope restriction to enforce."""
    entry = ACTION_CATALOG.get(action_type)
    return entry.scopes if entry is not None else None


def requires_capability(action_type: str) -> bool:
    """True for catalog actions gated by RobotConfig.supported_actions (the
    payload-handling/sensing capabilities: pick, drop, detectObject,
    finePositioning, waitForTrigger) — False for core protocol actions
    (always available) and for unknown/custom actionTypes (already gated by
    the caller's own supported_actions check)."""
    entry = ACTION_CATALOG.get(action_type)
    return entry is not None and not entry.core
