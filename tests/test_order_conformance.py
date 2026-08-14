"""Order-validation conformance: idle-gate, orderUpdateId rules, blockingType,
and unreleased-horizon gating — the exact VDA5050 rules confirmed against the
spec text (see plan / README), not the shallow approximation most reference
simulators implement.
"""

from __future__ import annotations

import asyncio

from helpers import (
    TEST_PREFIX,
    collect_states,
    make_action,
    make_node,
    make_order,
    make_route,
    publish_order,
    state_listener,
)

from vda5050_sim.schemas import BlockingType

MODEL, SERIAL = "spot", "test-spot-01"


async def test_new_order_while_idle_is_accepted(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (0.05, 0.0)])
    order = make_order(order_id="order-idle-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 3)
    assert states[-1].orderId == "order-idle-1"
    assert states[-1].orderUpdateId == 0


async def test_new_order_while_busy_is_rejected(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (50.0, 0.0)])
    busy_order = make_order(order_id="busy-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, busy_order)
    await asyncio.sleep(0.3)  # let it start driving (well short of a 50m trip)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        conflicting_order = make_order(
            order_id="conflict-2",
            order_update_id=0,
            model=MODEL,
            serial=SERIAL,
            nodes=[make_node("m0", 0, 5.0, 5.0)],
            edges=[],
        )
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, conflicting_order)

        matched = await listener.wait_for(lambda s: any(e.errorType == "otherOrderActive" for e in s.errors))
        assert matched.orderId == "busy-1"  # original order kept running, unaffected


async def test_order_update_with_higher_update_id_is_accepted(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (0.05, 0.0)])
    base = make_order(order_id="update-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, base)
    await asyncio.sleep(0.1)

    ext_nodes, ext_edges = make_route([(0.0, 0.0), (0.05, 0.0), (0.10, 0.0)])
    extended = make_order(
        order_id="update-1", order_update_id=1, model=MODEL, serial=SERIAL, nodes=ext_nodes, edges=ext_edges
    )
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, extended)

    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 3)
    assert states[-1].orderId == "update-1"
    assert states[-1].orderUpdateId == 1
    assert not any(e.errorType in ("outdatedOrderUpdate", "sameOrderUpdateId") for s in states for e in s.errors)


async def test_outdated_order_update_is_rejected(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (0.05, 0.0)])
    base = make_order(order_id="outdated-1", order_update_id=2, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, base)
    await asyncio.sleep(0.1)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        stale = make_order(
            order_id="outdated-1",
            order_update_id=1,  # lower than current (2)
            model=MODEL,
            serial=SERIAL,
            nodes=[make_node("n0", 0, 0.0, 0.0)],
            edges=[],
        )
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, stale)

        matched = await listener.wait_for(lambda s: any(e.errorType == "outdatedOrderUpdate" for e in s.errors))
        assert matched.orderUpdateId == 2  # unaffected


async def test_same_update_id_identical_content_is_ignored(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (0.05, 0.0)])
    order = make_order(order_id="dup-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges, header_id=1)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
    await asyncio.sleep(0.1)

    # Same orderId/orderUpdateId, identical graph, different headerId only —
    # headerId is excluded from the content-equality check.
    same_order_new_header = order.model_copy(update={"headerId": 2})

    states_before = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, same_order_new_header)
    states_after = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 3)

    assert not any(e.errorType == "sameOrderUpdateId" for s in states_after for e in s.errors)
    assert states_before[-1].orderId == states_after[-1].orderId == "dup-1"


async def test_same_update_id_different_content_is_rejected(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (0.05, 0.0)])
    order = make_order(
        order_id="conflict-update-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges
    )
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
    await asyncio.sleep(0.1)

    async with state_listener(fm, TEST_PREFIX, MODEL, SERIAL) as listener:
        conflicting_nodes, conflicting_edges = make_route([(0.0, 0.0), (9.0, 9.0)])
        conflicting = make_order(
            order_id="conflict-update-1",  # same id
            order_update_id=0,  # same update id, different graph
            model=MODEL,
            serial=SERIAL,
            nodes=conflicting_nodes,
            edges=conflicting_edges,
        )
        await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, conflicting)

        await listener.wait_for(lambda s: any(e.errorType == "sameOrderUpdateId" for e in s.errors))


async def test_hard_blocking_action_blocks_movement(running_fleet, fm):
    nodes, edges = make_route(
        [(0.0, 0.0), (0.05, 0.0)],
        node_actions={0: [make_action("a0", "pick", blocking=BlockingType.HARD)]},
    )
    order = make_order(order_id="hard-block-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    # action_duration_s=0.1 in test settings — sample early, movement must be
    # held at the start node until the HARD action finishes.
    await asyncio.sleep(0.05)
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].lastNodeId in ("", "n0")
    assert states[-1].mobileRobotPosition.x < 0.01  # hasn't moved toward n1 yet

    await asyncio.sleep(0.5)  # let the action (and the short trip) finish
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].lastNodeId == "n1"


async def test_unreleased_edge_is_never_traversed(running_fleet, fm):
    nodes, edges = make_route([(0.0, 0.0), (5.0, 0.0)], edge_released=[False])
    order = make_order(order_id="unreleased-1", order_update_id=0, model=MODEL, serial=SERIAL, nodes=nodes, edges=edges)
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)
    await asyncio.sleep(0.3)

    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 1)
    assert states[-1].driving is False
    assert states[-1].mobileRobotPosition.x == 0.0
    assert states[-1].newBaseRequest is True
