"""Strongest possible "complies with the standard" check: emitted messages
round-trip through the official VDA5050 3.0.0 JSON Schemas vendored in
src/vda5050_sim/json_schemas/, not just our own assumptions about the wire
shape. See that directory's README for the (upstream, not ours) syntax bugs
fixed at vendor time.

Two further upstream content issues remain, deliberately not patched, and
are asserted against directly here instead of via strict schema validation:
- `connection.schema`'s `connectionState` enum requires `CONNECTION_BROKEN`,
  while that same file's own prose describes `CONNECTIONBROKEN` — a
  self-contradiction in the official 3.0.0 release. We validate against the
  enum (see schemas.py's `ConnectionState`).
- `factsheet.schema`'s `typeSpecification.required` lists
  `mobileRobotKinematic` (no trailing "s"), which does not match the actual
  `mobileRobotKinematics` property — so no factsheet can ever pass strict
  validation for that object. Checked structurally instead.
"""

from __future__ import annotations

from helpers import TEST_PREFIX, collect_states, collect_visualizations, connection_listener

from vda5050_sim.agv import RobotConfig
from vda5050_sim.schemas import ConnectionState
from vda5050_sim.validate import validate_message

MODEL, SERIAL = "spot", "test-spot-01"


async def test_state_messages_conform_to_json_schema(running_fleet, fm):
    states = await collect_states(fm, TEST_PREFIX, MODEL, SERIAL, 2)
    for s in states:
        errors = validate_message(s.model_dump(mode="json", exclude_none=True), "state")
        assert errors == [], errors


async def test_visualization_messages_conform_to_json_schema(running_fleet, fm):
    viz = await collect_visualizations(fm, TEST_PREFIX, MODEL, SERIAL, 2)
    for v in viz:
        errors = validate_message(v.model_dump(mode="json", exclude_none=True), "visualization")
        assert errors == [], errors


async def test_connection_messages_conform_to_json_schema(fm, fleet_factory):
    serial = "schema-spot-01"
    async with connection_listener(fm, TEST_PREFIX, MODEL, serial) as listener:
        fleet = await fleet_factory([RobotConfig(id=serial, model=MODEL, supported_actions=["pick"])])
        await listener.wait_for(lambda c: c.connectionState == ConnectionState.ONLINE)

    for c in listener.received:
        errors = validate_message(c.model_dump(mode="json", exclude_none=True), "connection")
        assert errors == [], errors

    factsheet = fleet.runtimes[serial].agv.build_factsheet_message()
    assert factsheet.typeSpecification.seriesName == "Spot"
    assert factsheet.physicalParameters.maximumSpeed > 0
    assert any(a.actionType == "pick" for a in factsheet.protocolFeatures.mobileRobotActions)
