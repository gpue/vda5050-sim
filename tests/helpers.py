from __future__ import annotations

import asyncio

from vda5050_sim.robot_specs import get_manufacturer
from vda5050_sim.schemas import (
    Action,
    ActionParameter,
    BlockingType,
    ConnectionMessage,
    Corridor,
    Edge,
    FactsheetMessage,
    GrantType,
    InstantActionsMessage,
    Node,
    NodePosition,
    OrderMessage,
    OrientationType,
    Response,
    ResponsesMessage,
    StateMessage,
    Vertex2d,
    VisualizationMessage,
    Zone,
    ZoneSet,
    ZoneSetMessage,
    ZoneType,
)

TEST_PREFIX = "vda5050.v3test"


def make_node(node_id: str, seq: int, x: float, y: float, *, released: bool = True, actions=None) -> Node:
    return Node(
        nodeId=node_id,
        sequenceId=seq,
        released=released,
        nodePosition=NodePosition(x=x, y=y, mapId="default"),
        actions=actions or [],
    )


def make_edge(
    edge_id: str,
    seq: int,
    *,
    released: bool = True,
    actions=None,
    maximum_speed: float | None = None,
    orientation: float | None = None,
    orientation_type: OrientationType | None = None,
    reach_orientation_before_entering: bool | None = None,
    corridor: Corridor | None = None,
) -> Edge:
    # v3.0.0 edges have no startNodeId/endNodeId — traversal order comes
    # purely from the shared node/edge sequenceId space (see agv.py). Note:
    # there is also no separate "rotationAllowed" field in the real v3.0.0
    # schema — only reachOrientationBeforeEntering.
    return Edge(
        edgeId=edge_id,
        sequenceId=seq,
        released=released,
        actions=actions or [],
        maximumSpeed=maximum_speed,
        orientation=orientation,
        orientationType=orientation_type,
        reachOrientationBeforeEntering=reach_orientation_before_entering,
        corridor=corridor,
    )


def make_corridor(*, release_required: bool = False, left_width: float = 0.2, right_width: float = 0.2) -> Corridor:
    return Corridor(leftWidth=left_width, rightWidth=right_width, releaseRequired=release_required)


def make_action_param(action_id: str, action_type: str, params: dict, *, blocking: BlockingType = BlockingType.NONE) -> Action:
    return Action(
        actionId=action_id,
        actionType=action_type,
        blockingType=blocking,
        actionParameters=[ActionParameter(key=k, value=v) for k, v in params.items()],
    )


def make_route(
    positions: list[tuple[float, float]],
    *,
    node_ids: list[str] | None = None,
    edge_ids: list[str] | None = None,
    edge_released: list[bool] | None = None,
    node_actions: dict[int, list[Action]] | None = None,
) -> tuple[list[Node], list[Edge]]:
    """Build a straight-line node/edge chain with correctly interleaved
    sequenceIds (node=even, edge=odd) from a list of (x, y) positions."""
    n = len(positions)
    node_ids = node_ids or [f"n{i}" for i in range(n)]
    edge_ids = edge_ids or [f"e{i}" for i in range(n - 1)]
    edge_released = edge_released if edge_released is not None else [True] * (n - 1)
    node_actions = node_actions or {}

    nodes = [make_node(node_ids[i], i * 2, x, y, actions=node_actions.get(i)) for i, (x, y) in enumerate(positions)]
    edges = [make_edge(edge_ids[i], i * 2 + 1, released=edge_released[i]) for i in range(n - 1)]
    return nodes, edges


def make_action(
    action_id: str, action_type: str, *, blocking: BlockingType = BlockingType.NONE, retriable: bool | None = None
) -> Action:
    return Action(actionId=action_id, actionType=action_type, blockingType=blocking, retriable=retriable)


def make_order(
    *,
    order_id: str,
    order_update_id: int,
    model: str,
    serial: str,
    nodes: list[Node],
    edges: list[Edge],
    header_id: int = 1,
) -> OrderMessage:
    return OrderMessage(
        headerId=header_id,
        timestamp="2026-01-01T00:00:00.000Z",
        manufacturer=get_manufacturer(model),
        serialNumber=serial,
        orderId=order_id,
        orderUpdateId=order_update_id,
        nodes=nodes,
        edges=edges,
    )


def make_instant_actions(*, model: str, serial: str, actions: list[Action], header_id: int = 1) -> InstantActionsMessage:
    return InstantActionsMessage(
        headerId=header_id,
        timestamp="2026-01-01T00:00:00.000Z",
        manufacturer=get_manufacturer(model),
        serialNumber=serial,
        actions=actions,
    )


async def publish_order(fm, prefix: str, model: str, serial: str, order: OrderMessage) -> None:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.order"
    await fm.publish(subject, order.model_dump_json().encode())


async def publish_instant_actions(fm, prefix: str, model: str, serial: str, msg: InstantActionsMessage) -> None:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.instantActions"
    await fm.publish(subject, msg.model_dump_json().encode())


def make_zone(
    zone_id: str, zone_type: ZoneType, vertices: list[tuple[float, float]], **extra
) -> Zone:
    return Zone(zoneId=zone_id, zoneType=zone_type, vertices=[Vertex2d(x=x, y=y) for x, y in vertices], **extra)


def make_zone_set(*, map_id: str, zone_set_id: str, zones: list[Zone]) -> ZoneSet:
    return ZoneSet(mapId=map_id, zoneSetId=zone_set_id, zones=zones)


def make_zone_set_message(*, model: str, serial: str, zone_set: ZoneSet, header_id: int = 1) -> ZoneSetMessage:
    return ZoneSetMessage(
        headerId=header_id,
        timestamp="2026-01-01T00:00:00.000Z",
        manufacturer=get_manufacturer(model),
        serialNumber=serial,
        zoneSet=zone_set,
    )


def make_responses(
    *, model: str, serial: str, responses: list[tuple[str, GrantType]], header_id: int = 1
) -> ResponsesMessage:
    return ResponsesMessage(
        headerId=header_id,
        timestamp="2026-01-01T00:00:00.000Z",
        manufacturer=get_manufacturer(model),
        serialNumber=serial,
        responses=[Response(requestId=rid, grantType=gt) for rid, gt in responses],
    )


async def publish_zone_set(fm, prefix: str, model: str, serial: str, msg: ZoneSetMessage) -> None:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.zoneSet"
    await fm.publish(subject, msg.model_dump_json().encode())


async def publish_responses(fm, prefix: str, model: str, serial: str, msg: ResponsesMessage) -> None:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.responses"
    await fm.publish(subject, msg.model_dump_json().encode())


async def poll_until(predicate, timeout: float = 3.0, poll: float = 0.001):
    """Poll a synchronous predicate directly against live object state (e.g.
    a SimulatedAgv's own attributes) rather than a downstream NATS `state`
    snapshot — for transitions that can be transient enough (a single tick,
    ~ms) to fall between two periodic state publishes and never be observed
    that way, no matter how high state_hz is set."""
    loop = asyncio.get_event_loop()
    start = loop.time()
    while loop.time() - start < timeout:
        if predicate():
            return
        await asyncio.sleep(poll)
    raise AssertionError(f"timed out after {timeout}s waiting for condition to become true")


async def collect(fm, subject: str, parse, count: int, timeout: float = 5.0) -> list:
    """Subscribe to `subject`, collect `count` messages parsed by `parse`, then unsubscribe."""
    received: list = []
    done = asyncio.Event()

    async def _cb(msg) -> None:
        received.append(parse(msg.data))
        if len(received) >= count:
            done.set()

    sub = await fm.subscribe(subject, cb=_cb)
    try:
        await asyncio.wait_for(done.wait(), timeout=timeout)
    finally:
        await sub.unsubscribe()
    return received


async def collect_states(fm, prefix: str, model: str, serial: str, count: int, timeout: float = 5.0) -> list[StateMessage]:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.state"
    return await collect(fm, subject, StateMessage.model_validate_json, count, timeout)


async def collect_connections(fm, prefix: str, model: str, serial: str, count: int, timeout: float = 5.0) -> list[ConnectionMessage]:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.connection"
    return await collect(fm, subject, ConnectionMessage.model_validate_json, count, timeout)


async def collect_visualizations(
    fm, prefix: str, model: str, serial: str, count: int, timeout: float = 5.0
) -> list[VisualizationMessage]:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.visualization"
    return await collect(fm, subject, VisualizationMessage.model_validate_json, count, timeout)


async def collect_factsheets(fm, prefix: str, model: str, serial: str, count: int, timeout: float = 5.0) -> list[FactsheetMessage]:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.factsheet"
    return await collect(fm, subject, FactsheetMessage.model_validate_json, count, timeout)


class Listener:
    """Subscribe *before* triggering an event, then poll the growing buffer
    for a matching message — avoids races against a one-shot event (like a
    rejection error that appears in exactly one `state` publish)."""

    def __init__(self, fm, subject: str, parse) -> None:
        self.fm = fm
        self.subject = subject
        self.parse = parse
        self.received: list = []
        self._sub = None

    async def __aenter__(self) -> Listener:
        async def _cb(msg) -> None:
            self.received.append(self.parse(msg.data))

        self._sub = await self.fm.subscribe(self.subject, cb=_cb)
        # subscribe() only queues the SUB protocol frame locally — it does
        # NOT wait for the server to register it. flush() round-trips a
        # PING/PONG, guaranteeing the subscription is actually live before
        # the caller triggers whatever's supposed to publish to it. Every
        # existing use of this helper happened to have enough inherent
        # async delay between "enter the listener" and "the event actually
        # publishes" (an instant action round-tripping through a robot's own
        # task loop, etc.) to never expose this race — a debug endpoint that
        # publishes synchronously the moment it's called does not.
        await self.fm.flush()
        return self

    async def __aexit__(self, *exc) -> None:
        await self._sub.unsubscribe()

    async def wait_for(self, predicate, timeout: float = 3.0, poll: float = 0.02):
        loop = asyncio.get_event_loop()
        start = loop.time()
        while loop.time() - start < timeout:
            for item in self.received:
                if predicate(item):
                    return item
            await asyncio.sleep(poll)
        seen = len(self.received)
        raise AssertionError(f"timed out after {timeout}s waiting for a matching message on {self.subject} ({seen} seen)")


def state_listener(fm, prefix: str, model: str, serial: str) -> Listener:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.state"
    return Listener(fm, subject, StateMessage.model_validate_json)


def connection_listener(fm, prefix: str, model: str, serial: str) -> Listener:
    subject = f"{prefix}.{get_manufacturer(model)}.{serial}.connection"
    return Listener(fm, subject, ConnectionMessage.model_validate_json)
