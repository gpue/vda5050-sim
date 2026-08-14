from __future__ import annotations

import asyncio

from nova_vda5050 import (
    ConnectionMessage,
    FactsheetMessage,
    InstantActionsMessage,
    OrderMessage,
    StateMessage,
    VisualizationMessage,
    get_manufacturer,
)
from nova_vda5050.schemas import Action, BlockingType
from nova_vda5050.schemas.order import Edge, Node, NodePosition

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
    edge_id: str, seq: int, start: str, end: str, *, released: bool = True, actions=None
) -> Edge:
    return Edge(
        edgeId=edge_id,
        sequenceId=seq,
        released=released,
        startNodeId=start,
        endNodeId=end,
        actions=actions or [],
    )


def make_action(action_id: str, action_type: str, *, blocking: BlockingType = BlockingType.NONE) -> Action:
    return Action(actionId=action_id, actionType=action_type, blockingType=blocking)


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
