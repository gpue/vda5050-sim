"""A single simulated AGV: VDA5050 order/instant-action state machine + movement.

Implements the actual VDA5050 order-validation rules (idle-gate on new
orderId, orderUpdateId accept/reject/ignore, cancelOrder, pause, blockingType)
rather than the shallow approximation found in most reference simulators —
this is the part a fleet manager actually needs exercised correctly.

Movement follows the real v3.0.0 traversal model: nodes and edges share a
single `sequenceId` space (nodes at even positions, edges at odd positions
between them) — there is no `startNodeId`/`endNodeId` on Edge in v3.0.0.

Action progress (node-bound, edge-bound, and generic instant actions) is
advanced through one shared state machine (`_advance_one`/
`_advance_all_pending`, gated by `_is_blocking`/`_instant_action_is_blocking`)
keyed off a single `_action_registry` that outlives the owning node/edge
being dropped from the remaining-graph lists — this is what makes edge-bound
actions (previously never advanced at all) and non-blocking actions that
outlive their node/edge keep progressing correctly.
"""

from __future__ import annotations

import asyncio
import math
import random
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import UTC, datetime

from vda5050_sim import errors as sim_errors
from vda5050_sim.action_catalog import requires_capability, scopes_for
from vda5050_sim.robot_specs import build_factsheet, build_supported_actions, get_manufacturer
from vda5050_sim.schemas import (
    Action,
    ActionScope,
    ActionState,
    ActionStatus,
    BlockingType,
    ConnectionMessage,
    ConnectionState,
    Edge,
    EdgeRequest,
    EdgeRequestType,
    EdgeState,
    EmergencyStopType,
    Error,
    FactsheetMessage,
    GrantType,
    InstantActionsMessage,
    Load,
    MapState,
    MobileRobotPosition,
    Node,
    NodeState,
    OperatingMode,
    OrderMessage,
    OrientationType,
    PowerSupply,
    RequestStatus,
    Response,
    ResponsesMessage,
    SafetyState,
    StateMessage,
    Velocity,
    Vertex2d,
    VisualizationMessage,
    Zone,
    ZoneRequest,
    ZoneRequestType,
    ZoneSet,
    ZoneSetMessage,
    ZoneSetState,
    ZoneType,
)

LogFn = Callable[[str, str], None]

_ORIENTATION_TOLERANCE_RAD = math.radians(2)


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


def _angle_diff(target: float, current: float) -> float:
    """Signed shortest angular distance from `current` to `target`, in (-pi, pi]."""
    return (target - current + math.pi) % (2 * math.pi) - math.pi


def _point_in_polygon(x: float, y: float, vertices: list[Vertex2d]) -> bool:
    """Standard ray-casting point-in-polygon test."""
    n = len(vertices)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i].x, vertices[i].y
        xj, yj = vertices[j].x, vertices[j].y
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


_ZONE_TYPES_REQUIRING_ACCESS = (ZoneType.RELEASE, ZoneType.COORDINATED_REPLANNING)
_ZONE_DIRECTION_TOLERANCE_RAD = math.radians(45)

# Telemetry-realism tuning. Not spec-mandated durations — chosen to be long
# enough to observe/test against, short enough not to stall demos.
_FIELD_VIOLATION_DURATION_S = 1.0
_SERVICE_MODE_FAULT_DURATION_S = 2.0
_EMERGENCY_STOP_DURATION_S = 1.5


@dataclass
class FaultProfile:
    """Optional flakiness for resilience-testing a fleet manager."""

    connection_drop_probability: float = 0.0
    error_injection_probability: float = 0.0
    # Per real-world tick (not gated by pause/order state — see maybe_inject_fault).
    field_violation_probability: float = 0.0
    service_mode_probability: float = 0.0
    emergency_stop_probability: float = 0.0


@dataclass
class RobotConfig:
    id: str
    model: str
    manufacturer: str = ""
    supported_actions: list[str] = field(default_factory=list)
    max_speed: float | None = None
    angular_speed: float | None = None
    initial_battery: float = 100.0
    battery_drain_percent_per_meter: float | None = None
    battery_charge_percent_per_s: float | None = None
    # Home/dock position — every robot otherwise starts at the map origin,
    # which stacks an entire fleet on top of one another on any map view
    # until each has been individually dispatched somewhere.
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_theta: float = 0.0
    fault_profile: FaultProfile = field(default_factory=FaultProfile)
    # VDA5050 header `version` this robot announces itself as, and the wire
    # shape it publishes state/connection in — "3.0.0" (default) matches
    # this simulator's native internal model 1:1; "1.x"/"2.x" values are
    # downgraded at the transport boundary (see legacy_shapes.py) so nova-nav
    # and other consumers can be tested against a mixed-version fleet.
    protocol_version: str = "3.0.0"

    def __post_init__(self) -> None:
        if not self.manufacturer:
            self.manufacturer = get_manufacturer(self.model)


_BLOCKING_TYPES_THAT_HOLD_MOVEMENT = (BlockingType.HARD, BlockingType.SOFT, BlockingType.SINGLE)
_ACTIVE_ACTION_STATUSES = (ActionStatus.WAITING, ActionStatus.INITIALIZING, ActionStatus.RUNNING, ActionStatus.PAUSED)

# Instant actions that only exist from VDA5050 3.0.0 onward (per the
# project's own 3.0.0 release notes: zones, and the updateCertificate/
# trigger actions are all listed as new in that release) — a robot
# configured with an older `protocol_version` rejects these rather than
# silently supporting a capability its announced version doesn't have.
LEGACY_UNSUPPORTED_ACTIONS = frozenset(
    {"downloadZoneSet", "enableZoneSet", "deleteZoneSet", "updateCertificate", "trigger"}
)


class SimulatedAgv:
    """VDA5050 order/instant-action state machine for one robot."""

    def __init__(
        self,
        cfg: RobotConfig,
        *,
        action_duration_s: float,
        default_speed_mps: float,
        default_angular_speed_rad_s: float = 1.0,
        horizon_threshold_nodes: int = 2,
        default_battery_drain_percent_per_meter: float = 0.05,
        default_battery_charge_percent_per_s: float = 5.0,
        on_log: LogFn | None = None,
    ) -> None:
        self.cfg = cfg
        self.manufacturer = cfg.manufacturer
        self.serial_number = cfg.id
        self._action_duration_s = action_duration_s
        self._speed = cfg.max_speed or default_speed_mps
        self._angular_speed = cfg.angular_speed or default_angular_speed_rad_s
        self._horizon_threshold_nodes = horizon_threshold_nodes
        self._battery_drain_percent_per_meter = (
            cfg.battery_drain_percent_per_meter
            if cfg.battery_drain_percent_per_meter is not None
            else default_battery_drain_percent_per_meter
        )
        self._battery_charge_percent_per_s = (
            cfg.battery_charge_percent_per_s
            if cfg.battery_charge_percent_per_s is not None
            else default_battery_charge_percent_per_s
        )
        self._on_log: LogFn = on_log or (lambda *_: None)

        # Order/graph state — `nodes`/`edges` hold only the *remaining*
        # (not-yet-consumed) part of the graph, per VDA5050 state semantics.
        self.order_id = ""
        self.order_update_id = 0
        self._order_snapshot: OrderMessage | None = None
        self.nodes: list[Node] = []
        self.edges: list[Edge] = []
        self.last_node_id = ""
        self.last_node_sequence_id = 0
        self.action_states: list[ActionState] = []
        self.instant_action_states: list[ActionState] = []
        self.zone_action_states: list[ActionState] = []
        # Action definitions (blockingType etc.), keyed by actionId, kept
        # alive independent of whether the owning node/edge is still in
        # self.nodes/self.edges — lets non-blocking actions keep progressing
        # (and blocking ones keep being checked) after their node/edge is
        # consumed from the remaining graph.
        self._action_registry: dict[str, Action] = {}
        self.loads: list[Load] = []

        # Map / zone-set lifecycle (downloadMap/enableMap/deleteMap,
        # downloadZoneSet/enableZoneSet/deleteZoneSet — spec Section 6.3/6.4).
        # `"default"` is always implicitly present/enabled so existing
        # single-map demos/tests keep working without a fleet manager having
        # to explicitly download+enable it first.
        self._maps: dict[tuple[str, str], MapState] = {}
        self._zone_set_status: dict[str, str] = {}  # zoneSetId -> ENABLED/DISABLED

        # Hibernation / shutdown (startHibernation/stopHibernation/shutdown).
        self.hibernating = False
        self._wake_up_at: str | None = None
        self.shutdown_requested = False
        # Set by fleet.py's _state_loop each time it starts; stateRequest
        # wakes it early instead of waiting out the normal publish interval.
        self.state_request_event: asyncio.Event | None = None

        # Zone/traffic-control (Phase 3). Zone set definitions are map-scoped
        # fleet-wide knowledge — they persist across orders, unlike
        # zone_requests/edge_requests which are tied to the current order's
        # edges and reset on a fresh (non-update) order accept.
        self._zone_sets: dict[str, ZoneSet] = {}
        self.zone_requests: list[ZoneRequest] = []
        self.edge_requests: list[EdgeRequest] = []
        self._request_lease_expiry: dict[str, str] = {}
        # ACTION zones currently occupied (zoneId -> Zone), for entry/exit
        # action detection in _update_zone_actions.
        self._zone_occupancy: dict[str, Zone] = {}

        self.driving = False
        self.paused = False
        self.new_base_request = False
        self._cancel_requested_action_id: str | None = None
        self._action_started_at: dict[str, float] = {}
        self._elapsed_s = 0.0

        # Kinematics / telemetry
        self.x = cfg.initial_x
        self.y = cfg.initial_y
        self.theta = cfg.initial_theta
        self.map_id = "default"
        self.battery_soc = cfg.initial_battery
        self.charging = False
        self.emergency_stop = EmergencyStopType.NONE
        self._last_step_distance = 0.0

        # Telemetry realism: safety-field / operating-mode / e-stop faults
        # tick on real wall-clock time via maybe_inject_fault(dt) —
        # deliberately NOT gated by self.paused/_elapsed_s, since a physical
        # safety-field trip or mode change isn't suspended just because the
        # order is.
        self.field_violation = False
        self.operating_mode = OperatingMode.AUTOMATIC
        self._fault_elapsed_s = 0.0
        self._field_violation_until = 0.0
        self._operating_mode_fault_until = 0.0
        self._emergency_stop_until = 0.0
        self.distance_since_last_node = 0.0

        self.pending_errors: list[Error] = []
        self.factsheet_requested = True  # publish once at startup

        # VDA5050 convention: each message *type* has its own incrementing headerId.
        self._header_ids = {"state": 0, "visualization": 0, "connection": 0, "factsheet": 0}

    # -- header ids -----------------------------------------------------------

    def _next_header_id(self, kind: str) -> int:
        self._header_ids[kind] += 1
        return self._header_ids[kind]

    def _log(self, msg: str) -> None:
        self._on_log(self.serial_number, msg)

    # -- idle / content-equality helpers --------------------------------------

    def is_idle(self) -> bool:
        if self.driving:
            return False
        if self.nodes or self.edges:
            return False
        busy = set(_ACTIVE_ACTION_STATUSES)
        return not any(a.actionStatus in busy for a in self.action_states)

    def _order_content_equal(self, order: OrderMessage) -> bool:
        if self._order_snapshot is None:
            return False
        fields = {"orderId", "orderUpdateId", "nodes", "edges"}
        return self._order_snapshot.model_dump(include=fields) == order.model_dump(include=fields)

    # -- order handling ---------------------------------------------------------

    def handle_order(self, order: OrderMessage) -> None:
        if self.hibernating:
            # Spec: while HIBERNATING, "shall not respond to any other
            # commands, such as orders or additional instant actions."
            self._log(f"order '{order.orderId}' ignored — robot is HIBERNATING")
            return
        unknown_maps = {
            n.nodePosition.mapId
            for n in order.nodes
            if n.nodePosition is not None and n.nodePosition.mapId != "default" and not self._map_enabled(n.nodePosition.mapId)
        }
        for map_id in unknown_maps:
            self.pending_errors.append(
                sim_errors.make_error(sim_errors.UNKNOWN_MAP_ID, f"map '{map_id}' is not enabled on this robot")
            )
        if self.order_id and order.orderId == self.order_id:
            if order.orderUpdateId > self.order_update_id:
                self._log(f"order update accepted: {order.orderId}#{order.orderUpdateId}")
                self._accept_order(order, is_update=True)
            elif order.orderUpdateId == self.order_update_id:
                if self._order_content_equal(order):
                    self._log(f"duplicate order update ignored: {order.orderId}#{order.orderUpdateId}")
                    return
                self._log(f"order update rejected (sameOrderUpdateId): {order.orderId}#{order.orderUpdateId}")
                self.pending_errors.append(
                    sim_errors.make_error(
                        sim_errors.SAME_ORDER_UPDATE_ID,
                        f"orderUpdateId {order.orderUpdateId} already processed with different content",
                    )
                )
            else:
                self._log(f"order update rejected (outdatedOrderUpdate): {order.orderId}#{order.orderUpdateId}")
                self.pending_errors.append(
                    sim_errors.make_error(
                        sim_errors.OUTDATED_ORDER_UPDATE,
                        f"orderUpdateId {order.orderUpdateId} is lower than current {self.order_update_id}",
                    )
                )
            return

        if self.is_idle():
            self._log(f"order accepted: {order.orderId}#{order.orderUpdateId}")
            self._accept_order(order, is_update=False)
        else:
            self._log(f"order rejected (otherOrderActive): {order.orderId} while running {self.order_id}")
            self.pending_errors.append(
                sim_errors.make_error(sim_errors.OTHER_ORDER_ACTIVE, f"robot is executing order {self.order_id}")
            )

    def _accept_order(self, order: OrderMessage, *, is_update: bool) -> None:
        self.order_id = order.orderId
        self.order_update_id = order.orderUpdateId
        self._order_snapshot = order.model_copy(deep=True)
        self.nodes = list(order.nodes)
        self.edges = list(order.edges)
        if not is_update:
            self.last_node_id = ""
            self.last_node_sequence_id = 0
            self.action_states = []
            self.zone_requests = []
            self.edge_requests = []

        known_ids = {a.actionId for a in self.action_states}
        for node in self.nodes:
            for action in node.actions:
                if action.actionId in known_ids:
                    continue
                self._register_order_action(action, ActionScope.NODE)
                known_ids.add(action.actionId)
        for edge in self.edges:
            for action in edge.actions:
                if action.actionId in known_ids:
                    continue
                self._register_order_action(action, ActionScope.EDGE)
                known_ids.add(action.actionId)
        self.new_base_request = False

    def _reject_order_action(self, action: Action, reason: str) -> None:
        self._log(f"action '{action.actionType}' ({action.actionId}) rejected — {reason}")
        self.action_states.append(
            ActionState(actionId=action.actionId, actionType=action.actionType, actionStatus=ActionStatus.FAILED, actionResult=reason)
        )
        self.pending_errors.append(sim_errors.make_error(sim_errors.VALIDATION_ERROR, reason))

    def _register_order_action(self, action: Action, required_scope: ActionScope) -> None:
        """Register a node/edge-attached action's definition and initial
        state — rejecting up front (FAILED, no progression) if a known
        catalog actionType is used outside the scope Table 4 defines for it
        (e.g. `pick` on a node is fine, `stateRequest` on a node is not).
        Unknown actionTypes (manufacturer-custom) have no scope to violate."""
        scopes = scopes_for(action.actionType)
        if scopes is not None and required_scope not in scopes:
            self._reject_order_action(action, f"'{action.actionType}' is not a {required_scope.value}-scoped action")
            return
        # Known catalog actions are only capability-gated if core=False
        # (pick/drop/detectObject/finePositioning/waitForTrigger); unknown
        # (manufacturer-custom) actionTypes always require an explicit
        # supported_actions declaration — mirrors the instant-action path.
        capability_required = requires_capability(action.actionType) if scopes is not None else True
        if capability_required and action.actionType not in self.cfg.supported_actions:
            self._reject_order_action(action, f"'{action.actionType}' is not a supported capability of this robot")
            return
        self._action_registry[action.actionId] = action
        self.action_states.append(
            ActionState(actionId=action.actionId, actionType=action.actionType, actionStatus=ActionStatus.WAITING)
        )

    # -- instant actions ----------------------------------------------------

    # Catalog actions that resolve through the same generic WAITING->RUNNING->
    # FINISHED/FAILED timer machinery as node/edge actions (via
    # `_apply_action_effect`, dispatched from `_advance_one`/`_finish_action`)
    # rather than resolving synchronously the instant they're received.
    _TIMER_INSTANT_ACTIONS = frozenset(
        {
            "startCharging",
            "stopCharging",
            "initializePosition",
            "enableMap",
            "downloadMap",
            "deleteMap",
            "downloadZoneSet",
            "enableZoneSet",
            "deleteZoneSet",
            "updateCertificate",
        }
    )

    def handle_instant_actions(self, msg: InstantActionsMessage) -> None:
        for action in msg.actions:
            self._handle_instant_action(action)

    def _pause_or_resume(self, *, pause: bool) -> None:
        """Flip currently-RUNNING actions to/from PAUSED. Pausing is commanded
        instantaneously — no tick()-side bookkeeping needed; elapsed-time
        accrual is itself gated on `self.paused` in `tick()`, so a paused
        action's remaining duration budget is preserved exactly."""
        from_status, to_status = (ActionStatus.RUNNING, ActionStatus.PAUSED) if pause else (ActionStatus.PAUSED, ActionStatus.RUNNING)
        for s in (*self.action_states, *self.instant_action_states, *self.zone_action_states):
            if s.actionStatus == from_status:
                s.actionStatus = to_status

    def _reject_instant_action(self, action: Action, error_type: str, reason: str) -> None:
        self.instant_action_states.append(
            ActionState(actionId=action.actionId, actionType=action.actionType, actionStatus=ActionStatus.FAILED, actionResult=reason)
        )
        self.pending_errors.append(sim_errors.make_error(error_type, reason))

    def _find_tracked_action_state(self, action_id: str) -> ActionState | None:
        return next(
            (s for s in (*self.action_states, *self.instant_action_states, *self.zone_action_states) if s.actionId == action_id),
            None,
        )

    def _handle_instant_action(self, action: Action) -> None:
        at = action.actionType
        if at in LEGACY_UNSUPPORTED_ACTIONS and self.cfg.protocol_version.split(".", 1)[0] in ("1", "2"):
            self._log(f"instant action '{at}' rejected — requires VDA5050 3.0.0+, robot is {self.cfg.protocol_version}")
            self._reject_instant_action(
                action, sim_errors.INVALID_INSTANT_ACTION, f"'{at}' requires VDA5050 3.0.0+"
            )
            return
        if self.hibernating and at != "stopHibernation":
            # Spec: while HIBERNATING, "shall only receive and respond to the
            # instant action 'stopHibernation' and shall not respond to any
            # other commands" — silently ignored, not even a FAILED response.
            self._log(f"instant action '{at}' ignored — robot is HIBERNATING")
            return
        scopes = scopes_for(at)
        if scopes is not None and ActionScope.INSTANT not in scopes:
            self._log(f"instant action '{at}' rejected (wrong scope — not INSTANT-scoped)")
            self._reject_instant_action(action, sim_errors.VALIDATION_ERROR, f"'{at}' is not an INSTANT-scoped action")
            return
        params = {p.key: p.value for p in action.actionParameters}
        if at == "cancelOrder":
            if self.order_id and not self.is_idle():
                self._log(f"cancelOrder accepted for order {self.order_id}")
                self.instant_action_states.append(
                    ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.RUNNING)
                )
                self._cancel_requested_action_id = action.actionId
            else:
                self._reject_instant_action(action, sim_errors.NO_ORDER_TO_CANCEL, "cancelOrder received with no active order")
        elif at == "startPause":
            self.paused = True
            self._pause_or_resume(pause=True)
            self._log("paused (startPause)")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "stopPause":
            self.paused = False
            self._pause_or_resume(pause=False)
            self._log("resumed (stopPause)")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "factsheetRequest":
            self.factsheet_requested = True
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "stateRequest":
            if self.state_request_event is not None:
                self.state_request_event.set()
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "logReport":
            log_name = f"log_{self.serial_number}_{_now().replace(':', '-')}.txt"
            self._log(f"logReport generated: {log_name} (reason={params.get('reason', '-')})")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED, actionResult=log_name)
            )
        elif at == "clearInstantActions":
            before = len(self.instant_action_states)
            self.instant_action_states = [
                s for s in self.instant_action_states if s.actionStatus not in (ActionStatus.FINISHED, ActionStatus.FAILED)
            ]
            self._log(f"clearInstantActions: removed {before - len(self.instant_action_states)} terminal instant action(s)")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "clearZoneActions":
            before = len(self.zone_action_states)
            self.zone_action_states = [
                s for s in self.zone_action_states if s.actionStatus not in (ActionStatus.FINISHED, ActionStatus.FAILED)
            ]
            self._log(f"clearZoneActions: removed {before - len(self.zone_action_states)} terminal zone action(s)")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "startHibernation":
            self._clear_active_order()
            self.hibernating = True
            self._wake_up_at = str(params["wakeUpTime"]) if params.get("wakeUpTime") else None
            self._log("hibernation started" + (f" (wake at {self._wake_up_at})" if self._wake_up_at else ""))
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "stopHibernation":
            self.hibernating = False
            self._wake_up_at = None
            self._log("hibernation ended")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "shutdown":
            if self.is_idle():
                self.shutdown_requested = True
                self._log("shutdown accepted — going OFFLINE")
                self.instant_action_states.append(
                    ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
                )
            else:
                self._reject_instant_action(action, sim_errors.VALIDATION_ERROR, "shutdown requires an idle robot")
        elif at == "retry":
            target = self._find_tracked_action_state(str(params.get("actionId", "")))
            if target is None or target.actionStatus != ActionStatus.RETRIABLE:
                self._reject_instant_action(action, sim_errors.VALIDATION_ERROR, f"action '{params.get('actionId')}' is not RETRIABLE")
            else:
                target.actionStatus = ActionStatus.WAITING
                self._action_started_at.pop(target.actionId, None)
                self._log(f"action '{target.actionId}' retrying (via retry)")
                self.instant_action_states.append(
                    ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
                )
        elif at == "skipRetry":
            target = self._find_tracked_action_state(str(params.get("actionId", "")))
            if target is None or target.actionStatus != ActionStatus.RETRIABLE:
                self._reject_instant_action(action, sim_errors.VALIDATION_ERROR, f"action '{params.get('actionId')}' is not RETRIABLE")
            else:
                target.actionStatus = ActionStatus.FAILED
                target.actionResult = "skipped via skipRetry"
                self._log(f"action '{target.actionId}' skipped (via skipRetry)")
                self.instant_action_states.append(
                    ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
                )
        elif at == "trigger":
            # Table 4 lists no `actionId` param for `trigger` (unlike retry/
            # skipRetry, which do) — the spec doesn't fully define correlation
            # when multiple waitForTrigger actions are outstanding. Documented
            # judgment call: honor actionId if the sender supplies one anyway
            # (practical superset), else release the oldest RUNNING
            # waitForTrigger.
            target = self._find_tracked_action_state(str(params["actionId"])) if params.get("actionId") else None
            if target is None:
                target = self._oldest_running_wait_for_trigger()
            if target is None:
                self._reject_instant_action(action, sim_errors.VALIDATION_ERROR, "no waitForTrigger action is currently waiting")
            else:
                target.actionStatus = ActionStatus.FINISHED
                self._log(f"waitForTrigger '{target.actionId}' released (via trigger)")
                self.instant_action_states.append(
                    ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
                )
        elif at == "downloadMap":
            if self._start_download_map(action):
                self._register_timer_instant(action)
        elif at == "downloadZoneSet":
            if self._start_download_zone_set(action):
                self._register_timer_instant(action)
        elif at in self._TIMER_INSTANT_ACTIONS:
            self._register_timer_instant(action)
        elif at in self.cfg.supported_actions:
            # Manufacturer-custom action (not in the standard's Table 4
            # catalog) declared supported for this robot — same generic timer
            # machinery, no special-cased side effect on completion.
            self._register_timer_instant(action)
        else:
            self._reject_instant_action(action, sim_errors.VALIDATION_ERROR, f"unsupported instantAction '{at}'")

    def _register_timer_instant(self, action: Action) -> None:
        self._log(f"instant action '{action.actionType}' ({action.actionId}) accepted")
        self._action_registry[action.actionId] = action
        self.instant_action_states.append(
            ActionState(actionId=action.actionId, actionType=action.actionType, actionStatus=ActionStatus.WAITING)
        )

    def _oldest_running_wait_for_trigger(self) -> ActionState | None:
        for s in (*self.action_states, *self.zone_action_states):
            if s.actionStatus != ActionStatus.RUNNING:
                continue
            action_def = self._action_registry.get(s.actionId)
            if action_def is not None and action_def.actionType == "waitForTrigger":
                return s
        return None

    def _clear_active_order(self) -> None:
        """Shared cleanup for cancelOrder-finish and startHibernation (which
        must also clear any active order per spec)."""
        self.nodes = []
        self.edges = []
        self.action_states = []
        self.zone_requests = []
        self.edge_requests = []
        self.driving = False
        self.new_base_request = False
        self.order_id = ""
        self.order_update_id = 0
        self._order_snapshot = None

    def _finish_cancel(self) -> None:
        aid = self._cancel_requested_action_id
        for s in self.instant_action_states:
            if s.actionId == aid and s.actionStatus == ActionStatus.RUNNING:
                s.actionStatus = ActionStatus.FINISHED
                break
        self._log(f"cancelOrder finished — order {self.order_id} cleared, robot idle")
        self._clear_active_order()
        self._cancel_requested_action_id = None

    # -- map / zone-set lifecycle (downloadMap/enableMap/deleteMap,
    # downloadZoneSet/enableZoneSet/deleteZoneSet — spec Section 6.3/6.4) ------

    def _map_enabled(self, map_id: str) -> bool:
        if map_id == "default":
            return True
        return any(m.mapId == map_id and m.mapStatus == "ENABLED" for m in self._maps.values())

    def _start_download_map(self, action: Action) -> bool:
        params = {p.key: p.value for p in action.actionParameters}
        key = (str(params.get("mapId", "")), str(params.get("mapVersion", "")))
        if key in self._maps:
            self._reject_instant_action(action, sim_errors.DUPLICATE_MAP, f"map '{key[0]}'/{key[1]} already present")
            return False
        return True

    def _apply_download_map(self, action_def: Action) -> tuple[bool, str | None]:
        params = {p.key: p.value for p in action_def.actionParameters}
        map_id, map_version = str(params.get("mapId", "")), str(params.get("mapVersion", ""))
        self._maps[(map_id, map_version)] = MapState(mapId=map_id, mapVersion=map_version, mapStatus="DISABLED")
        return True, None

    def _apply_enable_map(self, action_def: Action) -> tuple[bool, str | None]:
        params = {p.key: p.value for p in action_def.actionParameters}
        map_id, map_version = str(params.get("mapId", "")), str(params.get("mapVersion", ""))
        if (map_id, map_version) not in self._maps:
            return False, f"map '{map_id}'/{map_version} not found"
        for (mid, mver), m in self._maps.items():
            if mid == map_id:
                m.mapStatus = "ENABLED" if mver == map_version else "DISABLED"
        return True, None

    def _apply_delete_map(self, action_def: Action) -> tuple[bool, str | None]:
        params = {p.key: p.value for p in action_def.actionParameters}
        key = (str(params.get("mapId", "")), str(params.get("mapVersion", "")))
        if key not in self._maps:
            return False, "map not found"
        del self._maps[key]
        return True, None

    def _start_download_zone_set(self, action: Action) -> bool:
        params = {p.key: p.value for p in action.actionParameters}
        zone_set_id = str(params.get("zoneSetId", ""))
        if zone_set_id in self._zone_sets:
            self._reject_instant_action(action, sim_errors.DUPLICATE_ZONE_SET, f"zone set '{zone_set_id}' already present")
            return False
        return True

    def _apply_download_zone_set(self, action_def: Action) -> tuple[bool, str | None]:
        params = {p.key: p.value for p in action_def.actionParameters}
        zone_set_id = str(params.get("zoneSetId", ""))
        # Pull-based download carries only a link, not geometry — this
        # simulator doesn't actually fetch anything, so it stores an empty
        # placeholder ZoneSet (documented simplification, same class as
        # updateCertificate's no-real-TLS simulation).
        self._zone_sets[zone_set_id] = ZoneSet(mapId=self.map_id, zoneSetId=zone_set_id, zones=[])
        self._zone_set_status[zone_set_id] = "DISABLED"
        return True, None

    def _apply_enable_zone_set(self, action_def: Action) -> tuple[bool, str | None]:
        params = {p.key: p.value for p in action_def.actionParameters}
        zone_set_id = str(params.get("zoneSetId", ""))
        zs = self._zone_sets.get(zone_set_id)
        if zs is None:
            return False, f"zone set '{zone_set_id}' not found"
        for zid, other in self._zone_sets.items():
            if other.mapId == zs.mapId:
                self._zone_set_status[zid] = "ENABLED" if zid == zone_set_id else "DISABLED"
        return True, None

    def _apply_delete_zone_set(self, action_def: Action) -> tuple[bool, str | None]:
        params = {p.key: p.value for p in action_def.actionParameters}
        zone_set_id = str(params.get("zoneSetId", ""))
        if zone_set_id not in self._zone_sets:
            return False, "zone set not found"
        del self._zone_sets[zone_set_id]
        self._zone_set_status.pop(zone_set_id, None)
        return True, None

    # -- load handling (pick/drop — spec Table 4) ------------------------------

    def _apply_pick(self, action_def: Action) -> tuple[bool, str | None]:
        params = {p.key: p.value for p in action_def.actionParameters}
        self.loads.append(
            Load(loadId=params.get("loadId"), loadType=params.get("loadType"), loadPosition=params.get("side"))
        )
        return True, None

    def _apply_drop(self, action_def: Action) -> tuple[bool, str | None]:
        if self.loads:
            self.loads.pop(0)
        return True, None

    def _apply_action_effect(self, action_def: Action) -> tuple[bool, str | None]:
        """Real state-mutating side effect for an action reaching FINISHED,
        dispatched from `_finish_action` — shared by instant, node, edge, and
        zone scope, since Table 4 lets several actionTypes (e.g.
        initializePosition, startCharging, enableMap) be sent via more than
        one scope with identical effects."""
        at = action_def.actionType
        if at == "startCharging":
            self.charging = True
            return True, None
        if at == "stopCharging":
            self.charging = False
            return True, None
        if at == "initializePosition":
            params = {p.key: p.value for p in action_def.actionParameters}
            try:
                self.x = float(params.get("x", self.x))
                self.y = float(params.get("y", self.y))
                self.theta = float(params.get("theta", self.theta))
                self.map_id = str(params.get("mapId", self.map_id))
                if "lastNodeId" in params:
                    self.last_node_id = str(params["lastNodeId"])
            except (TypeError, ValueError) as exc:
                return False, f"invalid initializePosition parameters: {exc}"
            return True, None
        if at == "enableMap":
            return self._apply_enable_map(action_def)
        if at == "downloadMap":
            return self._apply_download_map(action_def)
        if at == "deleteMap":
            return self._apply_delete_map(action_def)
        if at == "enableZoneSet":
            return self._apply_enable_zone_set(action_def)
        if at == "downloadZoneSet":
            return self._apply_download_zone_set(action_def)
        if at == "deleteZoneSet":
            return self._apply_delete_zone_set(action_def)
        if at == "pick":
            return self._apply_pick(action_def)
        if at == "drop":
            return self._apply_drop(action_def)
        if at == "clearInstantActions":
            # Only reachable here for a NODE-scoped invocation — the
            # INSTANT-scoped path resolves immediately in _handle_instant_action.
            self.instant_action_states = [
                s for s in self.instant_action_states if s.actionStatus not in (ActionStatus.FINISHED, ActionStatus.FAILED)
            ]
            return True, None
        if at == "clearZoneActions":
            self.zone_action_states = [
                s for s in self.zone_action_states if s.actionStatus not in (ActionStatus.FINISHED, ActionStatus.FAILED)
            ]
            return True, None
        return True, None  # detectObject/finePositioning/updateCertificate/custom: no extra state effect

    # -- zone/traffic-control (Phase 3) ----------------------------------------

    def handle_zone_set(self, msg: ZoneSetMessage) -> None:
        zone_set_id = msg.zoneSet.zoneSetId
        if zone_set_id in self._zone_sets:
            # Spec (line 917): duplicate zoneSetId is rejected the same way
            # whether delivered via this push topic or downloadZoneSet.
            self._log(f"zoneSet '{zone_set_id}' rejected — duplicate zoneSetId already stored")
            self.pending_errors.append(
                sim_errors.make_error(sim_errors.DUPLICATE_ZONE_SET, f"zone set '{zone_set_id}' already present")
            )
            return
        self._zone_sets[zone_set_id] = msg.zoneSet
        self._zone_set_status[zone_set_id] = "ENABLED"
        self._log(f"zoneSet '{zone_set_id}' received for map '{msg.zoneSet.mapId}' ({len(msg.zoneSet.zones)} zones)")

    def handle_responses(self, msg: ResponsesMessage) -> None:
        for resp in msg.responses:
            self._apply_response(resp)

    def _apply_response(self, resp: Response) -> None:
        # grantType (fleet control's reply) and requestStatus (the robot's own
        # view) are deliberately different enums — the spec doesn't state this
        # mapping explicitly; REJECTED has no dedicated robot-side status, so
        # it's treated the same as REVOKED (both mean "you may not proceed").
        status_for_grant = {
            GrantType.GRANTED: RequestStatus.GRANTED,
            GrantType.REVOKED: RequestStatus.REVOKED,
            GrantType.REJECTED: RequestStatus.REVOKED,
            GrantType.QUEUED: None,  # acknowledged, no permission yet — stays REQUESTED
        }
        new_status = status_for_grant[resp.grantType]
        matched = False
        for req in (*self.zone_requests, *self.edge_requests):
            if req.requestId != resp.requestId:
                continue
            matched = True
            if new_status is not None:
                req.requestStatus = new_status
            if resp.leaseExpiry:
                self._request_lease_expiry[req.requestId] = resp.leaseExpiry
            self._log(f"response for '{resp.requestId}': grantType={resp.grantType.value}")
        if not matched:
            self._log(f"response for unknown requestId '{resp.requestId}' ignored")

    def _check_lease_expiry(self) -> None:
        if not self._request_lease_expiry:
            return
        now = datetime.now(UTC)
        for req in (*self.zone_requests, *self.edge_requests):
            if req.requestStatus != RequestStatus.GRANTED:
                continue
            expiry_str = self._request_lease_expiry.get(req.requestId)
            if expiry_str is None:
                continue
            try:
                expiry = datetime.fromisoformat(expiry_str.replace("Z", "+00:00"))
            except ValueError:
                continue
            if now >= expiry:
                req.requestStatus = RequestStatus.EXPIRED
                self._log(f"request '{req.requestId}' lease expired")

    def _check_access_gates(self, edge: Edge, next_node: Node) -> bool:
        """Whether movement onto `edge`/`next_node` must hold for an
        outstanding corridor-release or zone-access grant. Creates the
        request (REQUESTED) the first time a gate is encountered."""
        holding = False

        if edge.corridor is not None and edge.corridor.releaseRequired:
            req = next((r for r in self.edge_requests if r.edgeId == edge.edgeId and r.sequenceId == edge.sequenceId), None)
            if req is None:
                req = EdgeRequest(
                    requestId=f"{edge.edgeId}-corridor",
                    requestType=EdgeRequestType.CORRIDOR,
                    edgeId=edge.edgeId,
                    sequenceId=edge.sequenceId,
                    requestStatus=RequestStatus.REQUESTED,
                )
                self.edge_requests.append(req)
                self._log(f"edge '{edge.edgeId}' requires corridor release — requested ({req.requestId})")
            if req.requestStatus != RequestStatus.GRANTED:
                holding = True

        if next_node.nodePosition is not None:
            zones_ahead = self._zone_objects_containing(next_node.nodePosition.x, next_node.nodePosition.y)
            for zone in zones_ahead:
                zone_type, zone_id = zone.zoneType, zone.zoneId
                zone_set_id = self._zone_set_id_for(zone_id)
                if zone_type in _ZONE_TYPES_REQUIRING_ACCESS:
                    req = next((r for r in self.zone_requests if r.zoneId == zone_id and r.zoneSetId == zone_set_id), None)
                    if req is None:
                        is_replanning = zone_type == ZoneType.COORDINATED_REPLANNING
                        request_type = ZoneRequestType.REPLANNING if is_replanning else ZoneRequestType.ACCESS
                        req = ZoneRequest(
                            requestId=f"{zone_id}-access",
                            requestType=request_type,
                            zoneId=zone_id,
                            zoneSetId=zone_set_id,
                            requestStatus=RequestStatus.REQUESTED,
                        )
                        self.zone_requests.append(req)
                        self._log(f"node near zone '{zone_id}' ({zone_type.value}) requires access — requested ({req.requestId})")
                    if req.requestStatus != RequestStatus.GRANTED:
                        holding = True
                elif zone_type == ZoneType.BLOCKED:
                    # No request/grant mechanism exists for BLOCKED in the spec
                    # — it means "do not enter", so fleet control must replan
                    # (newBaseRequest), not that a grant can ever arrive.
                    if not self.new_base_request:
                        self._log(f"next node lies in BLOCKED zone '{zone_id}' — requesting new base")
                    self.new_base_request = True
                    holding = True
                elif zone_type in (ZoneType.DIRECTED, ZoneType.BIDIRECTED) and zone.direction is not None:
                    heading = math.atan2(
                        next_node.nodePosition.y - self.y, next_node.nodePosition.x - self.x
                    )
                    if self._zone_direction_violation(zone, heading):
                        if zone.limitation in ("STRICT", "RESTRICTED"):
                            self._log(
                                f"movement into zone '{zone_id}' violates {zone_type.value} restriction "
                                f"({zone.limitation}) — holding, requesting new base"
                            )
                            self.new_base_request = True
                            holding = True
                        else:
                            self._log(f"movement into zone '{zone_id}' violates {zone_type.value} restriction (SOFT)")

        return holding

    def _zone_set_id_for(self, zone_id: str) -> str:
        for zone_set in self._zone_sets.values():
            if any(z.zoneId == zone_id for z in zone_set.zones):
                return zone_set.zoneSetId
        return ""

    def _zone_objects_containing(self, x: float, y: float) -> list[Zone]:
        """Zone objects (of any type) whose polygon contains (x, y) on the
        robot's current map."""
        hits = []
        for zone_set in self._zone_sets.values():
            if zone_set.mapId != self.map_id:
                continue
            for zone in zone_set.zones:
                if _point_in_polygon(x, y, zone.vertices):
                    hits.append(zone)
        return hits

    def _zone_direction_violation(self, zone: Zone, heading: float) -> bool:
        if zone.direction is None:
            return False
        diff = abs(_angle_diff(zone.direction, heading))
        if zone.zoneType == ZoneType.BIDIRECTED:
            diff = min(diff, abs(_angle_diff(zone.direction + math.pi, heading)))
        return diff > _ZONE_DIRECTION_TOLERANCE_RAD

    def _update_zone_actions(self) -> None:
        """ACTION zone entry/during/exit actions (spec Section 6.4.1) — fired
        through the same shared action-registry/advance machinery as node/
        edge/instant actions, tracked in `zone_action_states`."""
        present = {z.zoneId: z for z in self._zone_objects_containing(self.x, self.y) if z.zoneType == ZoneType.ACTION}
        for zone_id, zone in present.items():
            if zone_id in self._zone_occupancy:
                continue
            for action in (*zone.entryActions, *zone.duringActions):
                self._register_zone_action(action)
        for zone_id, zone in self._zone_occupancy.items():
            if zone_id in present:
                continue
            for action in zone.exitActions:
                self._register_zone_action(action)
        self._zone_occupancy = present

    def _register_zone_action(self, action: Action) -> None:
        if action.actionId in self._action_registry:
            return
        scopes = scopes_for(action.actionType)
        if scopes is not None and ActionScope.ZONE not in scopes:
            self.zone_action_states.append(
                ActionState(
                    actionId=action.actionId,
                    actionType=action.actionType,
                    actionStatus=ActionStatus.FAILED,
                    actionResult=f"'{action.actionType}' is not a ZONE-scoped action",
                )
            )
            return
        capability_required = requires_capability(action.actionType) if scopes is not None else True
        if capability_required and action.actionType not in self.cfg.supported_actions:
            self.zone_action_states.append(
                ActionState(
                    actionId=action.actionId,
                    actionType=action.actionType,
                    actionStatus=ActionStatus.FAILED,
                    actionResult=f"'{action.actionType}' is not a supported capability of this robot",
                )
            )
            return
        self._action_registry[action.actionId] = action
        self.zone_action_states.append(
            ActionState(actionId=action.actionId, actionType=action.actionType, actionStatus=ActionStatus.WAITING)
        )

    # -- unified action-progress state machine ---------------------------------
    #
    # One shared step advances node-bound, edge-bound, and generic instant
    # actions alike (WAITING->RUNNING->FINISHED against `_elapsed_s`, which
    # itself only accrues while not paused — see tick()).
    #
    # Progression and blocking are deliberately decoupled into two passes:
    # `_advance_all_pending` sweeps every tracked action every tick — via
    # `_action_registry`, not via the currently-referenced node/edge object —
    # so a NONE-blocking action started on a node/edge the robot has since
    # departed still keeps progressing instead of freezing at RUNNING forever
    # once that node/edge is dropped from the remaining graph. `_is_blocking`
    # then only inspects the *currently relevant* (current node + current
    # edge) action IDs to decide whether movement should hold — safe to scope
    # narrowly, since a HARD/SOFT/SINGLE action can never be left non-terminal
    # behind a departed node/edge: tick() never lets the robot depart while
    # one is still blocking.

    def _finish_action(self, state: ActionState, action_def: Action) -> None:
        ok, result = self._apply_action_effect(action_def)
        state.actionStatus = ActionStatus.FINISHED if ok else ActionStatus.FAILED
        if result:
            state.actionResult = result
        verb = "finished" if ok else "failed"
        self._log(f"action '{action_def.actionType}' ({state.actionId}) {verb}" + (f": {result}" if result else ""))

    def _advance_one(self, state: ActionState, action_def: Action) -> None:
        if state.actionStatus in (ActionStatus.FINISHED, ActionStatus.FAILED, ActionStatus.RETRIABLE):
            return  # RETRIABLE only leaves via an explicit retry/skipRetry instant action
        if state.actionStatus == ActionStatus.WAITING:
            state.actionStatus = ActionStatus.RUNNING
            self._action_started_at[state.actionId] = self._elapsed_s
            self._log(f"action '{action_def.actionType}' ({state.actionId}) running")
        if state.actionStatus == ActionStatus.RUNNING:
            if action_def.actionType == "waitForTrigger":
                return  # released externally by `trigger`, or FAILED on order cancel — never auto-times-out
            started = self._action_started_at.get(state.actionId, self._elapsed_s)
            if self._elapsed_s - started >= self._action_duration_s:
                self._finish_action(state, action_def)

    def _advance_all_pending(self) -> None:
        for state in (*self.action_states, *self.instant_action_states, *self.zone_action_states):
            action_def = self._action_registry.get(state.actionId)
            if action_def is not None:
                self._advance_one(state, action_def)

    def _is_blocking(self, action_ids: list[str]) -> bool:
        """Whether any of the given (currently node/edge-relevant) actions is
        non-terminal and its blockingType holds movement."""
        non_terminal = (ActionStatus.WAITING, ActionStatus.INITIALIZING, ActionStatus.RUNNING, ActionStatus.RETRIABLE)
        for action_id in action_ids:
            action_def = self._action_registry.get(action_id)
            state = next((s for s in self.action_states if s.actionId == action_id), None)
            if action_def is None or state is None:
                continue
            if state.actionStatus in non_terminal and action_def.blockingType in _BLOCKING_TYPES_THAT_HOLD_MOVEMENT:
                return True
        return False

    def _instant_action_is_blocking(self) -> bool:
        """Generic (non-control-plane) instant actions aren't tied to a node/
        edge — a blocking one holds movement regardless of graph position."""
        non_terminal = (ActionStatus.WAITING, ActionStatus.INITIALIZING, ActionStatus.RUNNING, ActionStatus.RETRIABLE)
        for state in self.instant_action_states:
            action_def = self._action_registry.get(state.actionId)
            if action_def is None:
                continue  # cancelOrder/pause/factsheetRequest/hibernation/etc. — not registry-tracked
            if state.actionStatus in non_terminal and action_def.blockingType in _BLOCKING_TYPES_THAT_HOLD_MOVEMENT:
                return True
        return False

    # -- movement engine ------------------------------------------------------
    #
    # v3.0.0 has no startNodeId/endNodeId on Edge: nodes and edges share one
    # sequenceId space (node, edge, node, edge, ...), so the edge that follows
    # a node has sequenceId == node.sequenceId + 1, and the node that follows
    # that edge has sequenceId == edge.sequenceId + 1.

    def _find_node_by_seq(self, seq: int) -> Node | None:
        return next((n for n in self.nodes if n.sequenceId == seq), None)

    def _find_edge_by_seq(self, seq: int) -> Edge | None:
        return next((e for e in self.edges if e.sequenceId == seq), None)

    def _current_node(self) -> Node | None:
        node = self._find_node_by_seq(self.last_node_sequence_id)
        if node is not None:
            return node
        return min(self.nodes, key=lambda n: n.sequenceId) if self.nodes else None

    def _resolve_edge_orientation(self, edge: Edge, heading_to_target: float) -> float | None:
        if edge.orientation is None:
            return None
        if edge.orientationType == OrientationType.GLOBAL:
            return edge.orientation
        # TANGENTIAL, or unspecified-but-orientation-given: treat as a
        # tangential offset per the spec's own example framing (0 = forwards,
        # PI = backwards relative to direction of travel).
        return heading_to_target + edge.orientation

    def _step_towards(self, node: Node, dt: float, *, edge: Edge | None = None) -> bool:
        self._last_step_distance = 0.0
        pos = node.nodePosition
        if pos is None:
            return True
        dx, dy = pos.x - self.x, pos.y - self.y
        dist = math.hypot(dx, dy)
        heading_to_target = math.atan2(dy, dx) if dist > 1e-9 else self.theta

        desired_theta: float | None = None
        must_pre_rotate = False
        effective_speed = self._speed
        if edge is not None:
            desired_theta = self._resolve_edge_orientation(edge, heading_to_target)
            must_pre_rotate = desired_theta is not None and bool(edge.reachOrientationBeforeEntering)
            if edge.maximumSpeed is not None:
                effective_speed = min(self._speed, edge.maximumSpeed)
        for zone in self._zone_objects_containing(self.x, self.y):
            if zone.zoneType == ZoneType.SPEED_LIMIT and zone.maximumSpeed is not None:
                effective_speed = min(effective_speed, zone.maximumSpeed)

        if must_pre_rotate and desired_theta is not None:
            delta = _angle_diff(desired_theta, self.theta)
            if abs(delta) > _ORIENTATION_TOLERANCE_RAD:
                step_theta = max(-self._angular_speed * dt, min(self._angular_speed * dt, delta))
                self.theta += step_theta
                return False  # still rotating in place — not yet arrived, hasn't moved
            self.theta = desired_theta

        step = effective_speed * dt
        if dist <= max(step, 1e-6):
            self._last_step_distance = dist
            self.x, self.y = pos.x, pos.y
            if desired_theta is not None:
                self.theta = desired_theta
            elif pos.theta is not None:
                self.theta = pos.theta
            self.map_id = pos.mapId
            return True

        self._last_step_distance = step
        self.theta = desired_theta if desired_theta is not None else heading_to_target
        self.x += dx / dist * step
        self.y += dy / dist * step
        return False

    def _arrive_at(self, node: Node, edge: Edge, *, departed_seq: int) -> None:
        self.last_node_id = node.nodeId
        self.last_node_sequence_id = node.sequenceId
        self.distance_since_last_node = 0.0
        self._log(f"arrived at node '{node.nodeId}' (seq {node.sequenceId})")
        self.edges = [e for e in self.edges if e.sequenceId != edge.sequenceId]
        self.nodes = [n for n in self.nodes if n.sequenceId != departed_seq]
        self.driving = False

    def _maybe_request_new_base(self) -> None:
        if self.new_base_request or not self.nodes:
            return
        released_remaining = sum(1 for n in self.nodes if n.released)
        if released_remaining <= self._horizon_threshold_nodes:
            self.new_base_request = True
            self._log(f"newBaseRequest: only {released_remaining} released node(s) remaining")

    def _update_charging(self, dt: float) -> None:
        # Charging is orthogonal to order execution — ticks even while paused
        # or in a non-AUTOMATIC fault mode, unlike the order-graph/action
        # progress below. No physical charger-dock modeling: charging just
        # works wherever/whenever commanded (documented simplification).
        if self.charging and self.battery_soc < 100.0:
            self.battery_soc = min(100.0, self.battery_soc + self._battery_charge_percent_per_s * dt)

    def _maybe_auto_wake(self) -> None:
        if not self._wake_up_at:
            return
        try:
            wake_dt = datetime.fromisoformat(self._wake_up_at.replace("Z", "+00:00"))
        except ValueError:
            return
        if datetime.now(UTC) >= wake_dt:
            self.hibernating = False
            self._wake_up_at = None
            self._log("hibernation ended — autonomous wake-up")

    def tick(self, dt: float) -> None:
        if self.hibernating:
            # Spec: "shall not be moving" while HIBERNATING; a set wakeUpTime
            # can autonomously end hibernation and resume normal operation.
            self._maybe_auto_wake()
            return

        self._update_charging(dt)

        if self._cancel_requested_action_id is not None:
            self._finish_cancel()
            return

        if self.paused or self.operating_mode != OperatingMode.AUTOMATIC:
            # elapsed_s frozen below — action-duration budgets don't burn while
            # paused or while a fault has taken the robot off AUTOMATIC (a real
            # AGV stops honoring orders when not in automatic mode).
            return

        self._elapsed_s += dt
        # Always advance every tracked action first, regardless of whether the
        # node graph still has anything left — a non-blocking action started
        # on the final edge/node must keep progressing to FINISHED even after
        # self.nodes empties out, or it freezes at RUNNING forever once the
        # early-return below starts firing every tick.
        self._advance_all_pending()
        self._check_lease_expiry()
        self._update_zone_actions()

        if not self.nodes:
            self.driving = False
            return

        current_node = self._current_node()
        if current_node is None:
            self.driving = False
            return

        edge = self._find_edge_by_seq(current_node.sequenceId + 1)

        current_action_ids = [a.actionId for a in current_node.actions]
        if edge is not None:
            current_action_ids += [a.actionId for a in edge.actions]
        if (
            self._is_blocking(current_action_ids)
            or self._instant_action_is_blocking()
            or self.field_violation
            or self.emergency_stop != EmergencyStopType.NONE
        ):
            self.driving = False
            return

        self._maybe_request_new_base()

        if edge is None:
            # No further edge from here — order graph exhausted from this node.
            self.driving = False
            self.last_node_id = current_node.nodeId
            self.last_node_sequence_id = current_node.sequenceId
            self.nodes = [n for n in self.nodes if n.sequenceId != current_node.sequenceId]
            return

        if not edge.released:
            self.driving = False
            self.new_base_request = True
            return

        next_node = self._find_node_by_seq(edge.sequenceId + 1)
        if next_node is None or not next_node.released:
            self.driving = False
            self.new_base_request = True
            return

        if self._check_access_gates(edge, next_node):
            self.driving = False
            return

        self.driving = True
        arrived = self._step_towards(next_node, dt, edge=edge)
        if self._last_step_distance > 0.0:
            self.battery_soc = max(0.0, self.battery_soc - self._last_step_distance * self._battery_drain_percent_per_meter)
            self.distance_since_last_node += self._last_step_distance
        if arrived:
            self._arrive_at(next_node, edge, departed_seq=current_node.sequenceId)

    # -- fault injection --------------------------------------------------------
    #
    # Ticks on real wall-clock time via `_fault_elapsed_s`, independent of the
    # order-execution `_elapsed_s` (which freezes during pause/fault-mode) —
    # a safety-field trip or mode fault is an environmental/hardware event,
    # not suspended just because the order happens to be.

    def _update_field_violation(self) -> None:
        if self.field_violation:
            if self._fault_elapsed_s >= self._field_violation_until:
                self.field_violation = False
                self._log("safety field violation cleared")
            return
        prob = self.cfg.fault_profile.field_violation_probability
        if prob and random.random() < prob:
            self.field_violation = True
            self._field_violation_until = self._fault_elapsed_s + _FIELD_VIOLATION_DURATION_S
            self._log("simulated safety field violation")

    def _update_operating_mode_fault(self) -> None:
        if self.operating_mode != OperatingMode.AUTOMATIC:
            if self._fault_elapsed_s >= self._operating_mode_fault_until:
                self.operating_mode = OperatingMode.AUTOMATIC
                self._log("operating mode restored to AUTOMATIC")
            return
        prob = self.cfg.fault_profile.service_mode_probability
        if prob and random.random() < prob:
            self.operating_mode = OperatingMode.SERVICE
            self._operating_mode_fault_until = self._fault_elapsed_s + _SERVICE_MODE_FAULT_DURATION_S
            self._log("simulated drop to SERVICE mode — order execution suspended")

    def _update_emergency_stop(self) -> None:
        if self.emergency_stop != EmergencyStopType.NONE:
            if self._fault_elapsed_s >= self._emergency_stop_until:
                self.emergency_stop = EmergencyStopType.NONE
                self._log("emergency stop cleared")
            return
        prob = self.cfg.fault_profile.emergency_stop_probability
        if prob and random.random() < prob:
            self.emergency_stop = random.choice((EmergencyStopType.MANUAL, EmergencyStopType.REMOTE))
            self._emergency_stop_until = self._fault_elapsed_s + _EMERGENCY_STOP_DURATION_S
            self._log(f"simulated emergency stop ({self.emergency_stop.value})")

    def _find_running_retriable_action(self) -> ActionState | None:
        for state in (*self.action_states, *self.instant_action_states, *self.zone_action_states):
            if state.actionStatus != ActionStatus.RUNNING:
                continue
            action_def = self._action_registry.get(state.actionId)
            if action_def is not None and action_def.retriable:
                return state
        return None

    def maybe_inject_fault(self, dt: float) -> None:
        self._fault_elapsed_s += dt
        self._update_field_violation()
        self._update_operating_mode_fault()
        self._update_emergency_stop()

        prob = self.cfg.fault_profile.error_injection_probability
        if not (prob and random.random() < prob):
            return
        running_retriable = self._find_running_retriable_action()
        if running_retriable is not None and self.cfg.protocol_version.split(".", 1)[0] in ("1", "2"):
            # RETRIABLE was only added in 3.0.0 — a legacy robot has no
            # retry path, so a failure just fails outright.
            running_retriable.actionStatus = ActionStatus.FAILED
            self._log(f"action '{running_retriable.actionId}' failed (no RETRIABLE support pre-3.0)")
        elif running_retriable is not None:
            # Real spec state machine (line 1258-1274): a retriable action
            # that fails while RUNNING goes to RETRIABLE, not FAILED — it
            # only leaves that state via an explicit `retry`/`skipRetry`
            # instant action from fleet control (see _handle_instant_action),
            # never an automatic timer.
            running_retriable.actionStatus = ActionStatus.RETRIABLE
            self._log(f"action '{running_retriable.actionId}' failed — RETRIABLE (awaiting retry/skipRetry)")
        else:
            self.pending_errors.append(sim_errors.make_error(sim_errors.HARDWARE_FAULT, "simulated transient fault"))
            self._log("simulated fault injected")

    def should_drop_connection(self) -> bool:
        prob = self.cfg.fault_profile.connection_drop_probability
        return bool(prob and random.random() < prob)

    # -- message builders ---------------------------------------------------

    def build_state_message(self) -> StateMessage:
        errors = list(self.pending_errors)
        self.pending_errors.clear()
        node_states = [
            NodeState(
                nodeId=n.nodeId,
                sequenceId=n.sequenceId,
                released=n.released,
                nodeDescriptor=n.nodeDescriptor,
                nodePosition=n.nodePosition,
            )
            for n in self.nodes
        ]
        edge_states = [
            EdgeState(edgeId=e.edgeId, sequenceId=e.sequenceId, released=e.released, edgeDescriptor=e.edgeDescriptor)
            for e in self.edges
        ]
        zone_set_states = [
            ZoneSetState(zoneSetId=zs.zoneSetId, mapId=zs.mapId, zoneSetStatus=self._zone_set_status.get(zs.zoneSetId, "ENABLED"))
            for zs in self._zone_sets.values()
        ]
        return StateMessage(
            headerId=self._next_header_id("state"),
            timestamp=_now(),
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            orderId=self.order_id,
            orderUpdateId=self.order_update_id,
            lastNodeId=self.last_node_id,
            lastNodeSequenceId=self.last_node_sequence_id,
            driving=self.driving,
            paused=self.paused,
            newBaseRequest=self.new_base_request,
            mobileRobotPosition=MobileRobotPosition(x=self.x, y=self.y, theta=self.theta, mapId=self.map_id, localized=True),
            velocity=Velocity(),
            powerSupply=PowerSupply(stateOfCharge=self.battery_soc, charging=self.charging),
            operatingMode=self.operating_mode,
            safetyState=SafetyState(activeEmergencyStop=self.emergency_stop, fieldViolation=self.field_violation),
            nodeStates=node_states,
            edgeStates=edge_states,
            actionStates=list(self.action_states),
            instantActionStates=list(self.instant_action_states),
            zoneActionStates=list(self.zone_action_states),
            errors=errors,
            loads=list(self.loads),
            distanceSinceLastNode=self.distance_since_last_node,
            maps=list(self._maps.values()),
            zoneSets=zone_set_states,
            zoneRequests=list(self.zone_requests),
            edgeRequests=list(self.edge_requests),
        )

    def build_visualization_message(self, reference_state_header_id: int) -> VisualizationMessage:
        return VisualizationMessage(
            headerId=self._next_header_id("visualization"),
            timestamp=_now(),
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            referenceStateHeaderId=reference_state_header_id,
            mobileRobotPosition=MobileRobotPosition(x=self.x, y=self.y, theta=self.theta, mapId=self.map_id, localized=True),
            velocity=Velocity(),
        )

    def build_connection_message(self, state: ConnectionState) -> ConnectionMessage:
        return ConnectionMessage(
            headerId=self._next_header_id("connection"),
            timestamp=_now(),
            manufacturer=self.manufacturer,
            serialNumber=self.serial_number,
            connectionState=state,
        )

    def build_factsheet_message(self) -> FactsheetMessage:
        return build_factsheet(
            self.cfg.model,
            self.serial_number,
            self._next_header_id("factsheet"),
            _now(),
            supported_actions=build_supported_actions(self.cfg.supported_actions),
        )
