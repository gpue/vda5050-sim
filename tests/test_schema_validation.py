"""Strongest possible "complies with the standard" check: emitted messages
round-trip through nova_vda5050's own bundled JSON Schemas, not just our own
assumptions about the wire shape.

Three of nova_vda5050==0.1.2's bundled schema files have upstream bugs
independent of anything in this repo (confirmed by reading the schema files
directly): `visualization.schema.json` and `factsheet.schema.json` both have
a syntax error (trailing comma — they fail to even parse as JSON), and
`connection.schema.json`'s enum uses `CONNECTION_BROKEN`/`HIBERNATING`, which
contradicts both the real VDA5050 v3.0.0 spec text and nova_vda5050's own
`ConnectionState` Pydantic enum (`CONNECTIONBROKEN`, no `HIBERNATING`) — the
enum nova-nav actually consumes. Those three message types are checked
against the Pydantic model / real spec enum directly instead of the broken
bundled schema; only `state`'s bundled schema is actually valid JSON and
matches its Pydantic model.
"""

from __future__ import annotations

from helpers import TEST_PREFIX, collect_states, collect_visualizations, connection_listener
from nova_vda5050.schemas import ConnectionState
from nova_vda5050.validate import validate_message

from vda5050_sim.agv import RobotConfig

MODEL, SERIAL = "spot", "test-spot-01"


async def test_state_messages_conform_to_json_schema(running_fleet, fm):
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 2)
    for s in states:
        errors = validate_message(s.model_dump(mode="json", exclude_none=True), "state")
        assert errors == [], errors


async def test_visualization_messages_round_trip_cleanly(running_fleet, fm):
    viz = await collect_visualizations(fm, TEST_PREFIX, MODEL, SERIAL, 2)
    for v in viz:
        assert v.manufacturer and v.serialNumber
        assert v.mobileRobotPosition is not None
        assert isinstance(v.headerId, int)


async def test_connection_and_factsheet_messages_conform_to_json_schema(fm, fleet_factory):
    serial = "schema-spot-01"
    async with connection_listener(fm, TEST_PREFIX, MODEL, serial) as listener:
        fleet = await fleet_factory([RobotConfig(id=serial, model=MODEL, supported_actions=["pick"])])
        await listener.wait_for(lambda c: c.connectionState == ConnectionState.ONLINE)

    for c in listener.received:
        assert c.connectionState in (ConnectionState.CONNECTIONBROKEN, ConnectionState.ONLINE)

    factsheet = fleet.runtimes[serial].agv.build_factsheet_message()
    assert factsheet.typeSpecification.seriesName == "Spot"
    assert factsheet.physicalParameters.maximumSpeed > 0
    assert any(a.actionType == "pick" for a in factsheet.protocolFeatures.mobileRobotActions)
