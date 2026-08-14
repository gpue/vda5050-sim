"""Connection lifecycle, headerId monotonicity, and multi-robot isolation."""

from __future__ import annotations

from helpers import (
    TEST_PREFIX,
    collect_states,
    connection_listener,
    make_node,
    make_order,
    publish_order,
)

from vda5050_sim.agv import RobotConfig
from vda5050_sim.schemas import ConnectionState

MODEL, SERIAL = "spot", "test-spot-01"


async def test_connection_lifecycle_on_startup(fm, fleet_factory):
    serial = "lifecycle-spot-01"
    async with connection_listener(fm, TEST_PREFIX, MODEL, serial) as listener:
        await fleet_factory([RobotConfig(id=serial, model=MODEL, supported_actions=[])])
        broken = await listener.wait_for(lambda c: c.connectionState == ConnectionState.CONNECTION_BROKEN)
        online = await listener.wait_for(lambda c: c.connectionState == ConnectionState.ONLINE)
    assert broken.headerId < online.headerId


async def test_header_id_increments_monotonically(running_fleet, fm):
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 4)
    ids = [s.headerId for s in states]
    assert ids == sorted(ids)
    assert len(set(ids)) == len(ids)  # strictly increasing, no repeats/resets


async def test_multi_robot_isolation(running_fleet, fm):
    order = make_order(
        order_id="iso-1",
        order_update_id=0,
        model=MODEL,
        serial=SERIAL,
        nodes=[make_node("n0", 0, 0.0, 0.0)],
        edges=[],
    )
    await publish_order(fm, TEST_PREFIX, MODEL, SERIAL, order)

    spot_states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 2)
    go2_states = await collect_states(fm, TEST_PREFIX, "go2", "test-go2-01", 2)

    assert spot_states[-1].orderId == "iso-1"
    assert go2_states[-1].orderId == ""  # unaffected by an order sent to a different robot
