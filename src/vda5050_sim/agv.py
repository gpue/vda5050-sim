"""A single simulated AGV: VDA5050 order/instant-action state machine + movement.

Implements the actual VDA5050 order-validation rules (idle-gate on new
orderId, orderUpdateId accept/reject/ignore, cancelOrder, pause, blockingType)
rather than the shallow approximation found in most reference simulators —
this is the part a fleet manager actually needs exercised correctly.

Movement follows the real v3.0.0 traversal model: nodes and edges share a
single `sequenceId` space (nodes at even positions, edges at odd positions
between them) — there is no `startNodeId`/`endNodeId` on Edge in v3.0.0.
"""

from __future__ import annotations

import math
import random
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import UTC, datetime

from vda5050_sim import errors as sim_errors
from vda5050_sim.robot_specs import action_def, build_factsheet, get_manufacturer
from vda5050_sim.schemas import (
    Action,
    ActionState,
    ActionStatus,
    BlockingType,
    ConnectionMessage,
    ConnectionState,
    Edge,
    EdgeState,
    EmergencyStopType,
    Error,
    FactsheetMessage,
    InstantActionsMessage,
    MobileRobotPosition,
    Node,
    NodeState,
    OperatingMode,
    OrderMessage,
    PowerSupply,
    SafetyState,
    StateMessage,
    Velocity,
    VisualizationMessage,
)

LogFn = Callable[[str, str], None]


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


@dataclass
class FaultProfile:
    """Optional flakiness for resilience-testing a fleet manager."""

    connection_drop_probability: float = 0.0
    error_injection_probability: float = 0.0


@dataclass
class RobotConfig:
    id: str
    model: str
    manufacturer: str = ""
    supported_actions: list[str] = field(default_factory=list)
    max_speed: float | None = None
    initial_battery: float = 100.0
    fault_profile: FaultProfile = field(default_factory=FaultProfile)

    def __post_init__(self) -> None:
        if not self.manufacturer:
            self.manufacturer = get_manufacturer(self.model)


_BLOCKING_TYPES_THAT_HOLD_MOVEMENT = (BlockingType.HARD, BlockingType.SOFT, BlockingType.SINGLE)


class SimulatedAgv:
    """VDA5050 order/instant-action state machine for one robot."""

    def __init__(
        self,
        cfg: RobotConfig,
        *,
        action_duration_s: float,
        default_speed_mps: float,
        on_log: LogFn | None = None,
    ) -> None:
        self.cfg = cfg
        self.manufacturer = cfg.manufacturer
        self.serial_number = cfg.id
        self._action_duration_s = action_duration_s
        self._speed = cfg.max_speed or default_speed_mps
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

        self.driving = False
        self.paused = False
        self.new_base_request = False
        self._cancel_requested_action_id: str | None = None
        self._action_started_at: dict[str, float] = {}
        self._elapsed_s = 0.0

        # Kinematics / telemetry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.map_id = "default"
        self.battery_soc = cfg.initial_battery
        self.charging = False
        self.emergency_stop = EmergencyStopType.NONE

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
        busy = {
            ActionStatus.WAITING,
            ActionStatus.INITIALIZING,
            ActionStatus.RUNNING,
            ActionStatus.PAUSED,
        }
        return not any(a.actionStatus in busy for a in self.action_states)

    def _order_content_equal(self, order: OrderMessage) -> bool:
        if self._order_snapshot is None:
            return False
        fields = {"orderId", "orderUpdateId", "nodes", "edges"}
        return self._order_snapshot.model_dump(include=fields) == order.model_dump(include=fields)

    # -- order handling ---------------------------------------------------------

    def handle_order(self, order: OrderMessage) -> None:
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

        known_ids = {a.actionId for a in self.action_states}
        for node in self.nodes:
            for action in node.actions:
                if action.actionId not in known_ids:
                    self.action_states.append(
                        ActionState(actionId=action.actionId, actionType=action.actionType, actionStatus=ActionStatus.WAITING)
                    )
                    known_ids.add(action.actionId)
        for edge in self.edges:
            for action in edge.actions:
                if action.actionId not in known_ids:
                    self.action_states.append(
                        ActionState(actionId=action.actionId, actionType=action.actionType, actionStatus=ActionStatus.WAITING)
                    )
                    known_ids.add(action.actionId)
        self.new_base_request = False

    # -- instant actions ----------------------------------------------------

    def handle_instant_actions(self, msg: InstantActionsMessage) -> None:
        for action in msg.actions:
            self._handle_instant_action(action)

    def _handle_instant_action(self, action: Action) -> None:
        at = action.actionType
        if at == "cancelOrder":
            if self.order_id and not self.is_idle():
                self._log(f"cancelOrder accepted for order {self.order_id}")
                self.instant_action_states.append(
                    ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.RUNNING)
                )
                self._cancel_requested_action_id = action.actionId
            else:
                self._log("cancelOrder rejected (noOrderToCancel)")
                self.instant_action_states.append(
                    ActionState(
                        actionId=action.actionId,
                        actionType=at,
                        actionStatus=ActionStatus.FAILED,
                        actionResult="no active order to cancel",
                    )
                )
                self.pending_errors.append(
                    sim_errors.make_error(sim_errors.NO_ORDER_TO_CANCEL, "cancelOrder received with no active order")
                )
        elif at == "startPause":
            self.paused = True
            self._log("paused (startPause)")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "stopPause":
            self.paused = False
            self._log("resumed (stopPause)")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at == "factsheetRequest":
            self.factsheet_requested = True
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        elif at in self.cfg.supported_actions:
            self._log(f"instant action '{at}' finished")
            self.instant_action_states.append(
                ActionState(actionId=action.actionId, actionType=at, actionStatus=ActionStatus.FINISHED)
            )
        else:
            self._log(f"instant action '{at}' rejected (unsupported)")
            self.instant_action_states.append(
                ActionState(
                    actionId=action.actionId,
                    actionType=at,
                    actionStatus=ActionStatus.FAILED,
                    actionResult="unsupported action type",
                )
            )
            self.pending_errors.append(sim_errors.make_error(sim_errors.VALIDATION_ERROR, f"unsupported instantAction '{at}'"))

    def _finish_cancel(self) -> None:
        aid = self._cancel_requested_action_id
        for s in self.instant_action_states:
            if s.actionId == aid and s.actionStatus == ActionStatus.RUNNING:
                s.actionStatus = ActionStatus.FINISHED
                break
        self._log(f"cancelOrder finished — order {self.order_id} cleared, robot idle")
        self.nodes = []
        self.edges = []
        self.action_states = []
        self.driving = False
        self.new_base_request = False
        self.order_id = ""
        self.order_update_id = 0
        self._order_snapshot = None
        self._cancel_requested_action_id = None

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

    def _run_node_actions(self, node: Node) -> bool:
        """Advance node-bound actions one tick. Returns True if movement must hold."""
        blocking = False
        for action in node.actions:
            state = next((s for s in self.action_states if s.actionId == action.actionId), None)
            if state is None or state.actionStatus == ActionStatus.FINISHED:
                continue
            if state.actionStatus == ActionStatus.WAITING:
                state.actionStatus = ActionStatus.RUNNING
                self._action_started_at[action.actionId] = self._elapsed_s
                self._log(f"action '{action.actionType}' ({action.actionId}) running")
            if state.actionStatus == ActionStatus.RUNNING:
                started = self._action_started_at.get(action.actionId, self._elapsed_s)
                if self._elapsed_s - started >= self._action_duration_s:
                    state.actionStatus = ActionStatus.FINISHED
                    self._log(f"action '{action.actionType}' ({action.actionId}) finished")
                elif action.blockingType in _BLOCKING_TYPES_THAT_HOLD_MOVEMENT:
                    blocking = True
        return blocking

    def _step_towards(self, node: Node, dt: float) -> bool:
        pos = node.nodePosition
        if pos is None:
            return True
        dx, dy = pos.x - self.x, pos.y - self.y
        dist = math.hypot(dx, dy)
        step = self._speed * dt
        if dist <= max(step, 1e-6):
            self.x, self.y = pos.x, pos.y
            if pos.theta is not None:
                self.theta = pos.theta
            self.map_id = pos.mapId
            return True
        self.theta = math.atan2(dy, dx)
        self.x += dx / dist * step
        self.y += dy / dist * step
        return False

    def _arrive_at(self, node: Node, edge: Edge, *, departed_seq: int) -> None:
        self.last_node_id = node.nodeId
        self.last_node_sequence_id = node.sequenceId
        self._log(f"arrived at node '{node.nodeId}' (seq {node.sequenceId})")
        self.edges = [e for e in self.edges if e.sequenceId != edge.sequenceId]
        self.nodes = [n for n in self.nodes if n.sequenceId != departed_seq]
        self.driving = False

    def tick(self, dt: float) -> None:
        self._elapsed_s += dt

        if self._cancel_requested_action_id is not None:
            self._finish_cancel()
            return

        if self.paused:
            return

        if not self.nodes:
            self.driving = False
            return

        current_node = self._current_node()
        if current_node is not None and self._run_node_actions(current_node):
            self.driving = False
            return

        if current_node is None:
            self.driving = False
            return

        edge = self._find_edge_by_seq(current_node.sequenceId + 1)
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

        self.driving = True
        if self._step_towards(next_node, dt):
            self._arrive_at(next_node, edge, departed_seq=current_node.sequenceId)

    # -- fault injection --------------------------------------------------------

    def maybe_inject_fault(self) -> None:
        prob = self.cfg.fault_profile.error_injection_probability
        if prob and random.random() < prob:
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
            NodeState(nodeId=n.nodeId, sequenceId=n.sequenceId, released=n.released, nodeDescriptor=n.nodeDescriptor)
            for n in self.nodes
        ]
        edge_states = [
            EdgeState(edgeId=e.edgeId, sequenceId=e.sequenceId, released=e.released, edgeDescriptor=e.edgeDescriptor)
            for e in self.edges
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
            operatingMode=OperatingMode.AUTOMATIC,
            safetyState=SafetyState(activeEmergencyStop=self.emergency_stop, fieldViolation=False),
            nodeStates=node_states,
            edgeStates=edge_states,
            actionStates=list(self.action_states),
            instantActionStates=list(self.instant_action_states),
            errors=errors,
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
        base = [
            action_def("stop", "Emergency stop", blocking=["HARD"], pause_allowed="false", cancel_allowed="false"),
            action_def("enable", "Enable robot"),
            action_def("disable", "Disable robot", blocking=["HARD"]),
        ]
        custom = [action_def(a, a) for a in self.cfg.supported_actions]
        return build_factsheet(
            self.cfg.model,
            self.serial_number,
            self._next_header_id("factsheet"),
            _now(),
            supported_actions=base + custom,
        )
