# vda5050-sim

A standards-compliant [VDA5050](https://github.com/VDA5050/VDA5050) (v3.0.0) robot fleet simulator. It fakes a configurable fleet of AGVs — by default 5 real robot archetypes (Boston Dynamics Spot, Unitree Go2, PAL Robotics TIAGo, Unitree H1, SunFounder PiDog) — that speak the real order/instant-action/state/zone protocol, so you can point any VDA5050 fleet manager at it and watch it drive a graph, reject conflicting orders, cancel, pause, hibernate/shut down, manage maps and zone sets, pick/drop loads, wait for external triggers, wait for corridor/zone access grants, and recover from injected faults (including a spec-correct `RETRIABLE`/`retry`/`skipRetry` flow), without needing real hardware.

It can also simulate a **mixed-protocol-version fleet** — some robots on the current v3.0.0, others announcing themselves as 2.1.0/2.0.0/1.1.0 with the wire shape and reduced capability set a real robot on that firmware would actually have (see [Simulating older VDA5050 protocol versions](#simulating-older-vda5050-protocol-versions)) — useful for testing that a fleet manager correctly discovers, parses, and displays a fleet it doesn't fully control the firmware of.

Unlike a quick reference simulator, this implements the actual protocol rules from the spec rather than a shallow approximation, verified by a conformance test suite:
- Order lifecycle: idle-gate on new orders, `orderUpdateId` accept/reject/ignore semantics, `cancelOrder`, pause, `blockingType` handling (on node, edge, *and* generic instant actions).
- The **full Section 6.2.3 "Predefined Actions" catalog** (`src/vda5050_sim/action_catalog.py`, transcribed directly from the spec's Table 4) — all ~28 standardized actionTypes, each with the exact instant/node/edge/zone scope the spec defines for it, enforced at runtime (a `pick` sent as an instant action, or a `stateRequest` sent on a node, is rejected the same way an unsupported action is). Covers connection-state lifecycle (`startHibernation`/`stopHibernation`/`shutdown`), map and zone-set management (`downloadMap`/`enableMap`/`deleteMap`, `downloadZoneSet`/`enableZoneSet`/`deleteZoneSet`), load handling (`pick`/`drop`, reflected in `state.loads`), sensing (`detectObject`/`finePositioning`), external correlation (`waitForTrigger`/`trigger`), housekeeping (`clearInstantActions`/`clearZoneActions`/`stateRequest`/`logReport`), and a real `RETRIABLE` action-state machine (`retry`/`skipRetry`), not just the handful of actions most reference simulators implement.
- Movement fidelity: per-edge `maximumSpeed`/orientation/`reachOrientationBeforeEntering`, proactive horizon (`newBaseRequest`) extension, `distanceSinceLastNode` tracking.
- Traffic control: `Edge.corridor.releaseRequired` and `RELEASE`/`COORDINATED_REPLANNING` zones gate movement on a `responses` grant from fleet control, exactly as the spec defines. Five of VDA5050's ten zone types have real simulated runtime effects: `BLOCKED` (holds movement, requests a new base), `SPEED_LIMIT` (caps travel speed inside the zone), `ACTION` (fires entry/during/exit actions), `DIRECTED`/`BIDIRECTED` (holds movement against a restricted travel direction).
- Real MQTT semantics: one MQTT connection per robot (matching how real AGVs connect), each with its own retained messages and Last-Will-Testament, so a killed robot process is auto-reported `CONNECTION_BROKEN` by the broker itself — not just on clean shutdown.

![Log-viewer UI showing the default 5-robot fleet and live event log](docs/screenshot.png)

## Two ways to run it

**1. Standalone — plain VDA5050-over-MQTT (recommended default).**
This is the real VDA5050 wire protocol, so it works with any standard fleet manager / master control, no other infrastructure required:

```bash
docker build -t vda5050-sim .
docker run -p 8000:8000 \
  -e TRANSPORT=mqtt \
  -e MQTT_HOST=your-broker-host \
  -e MQTT_PORT=1883 \
  vda5050-sim
```

Each robot opens its **own** MQTT connection (own Last-Will-Testament), publishing/subscribing on the standard topic layout `vda5050/v3/{manufacturer}/{serialNumber}/{topic}` (`order`, `instantActions`, `zoneSet`, `responses`, `state`, `visualization`, `connection`, `factsheet`) against whatever broker you point it at (Mosquitto, EMQX, HiveMQ, your fleet manager's own embedded broker, etc.). `state`/`connection`/`factsheet` are published with `retain=True` (`visualization` deliberately isn't — high-rate/ephemeral). The log-viewer UI is at `http://localhost:8000/`.

**2. Over NATS.**
Set `TRANSPORT=nats` (and `NATS_BROKER=nats://...`) if your own stack already runs on a NATS message bus instead of MQTT — the same simulation logic runs either way, just on `vda5050.v3.{manufacturer}.{serialNumber}.{topic}` subjects over one shared connection instead of per-robot MQTT connections (core NATS has no retain/LWT concept to motivate the per-robot split there).

## Connecting a real fleet manager

Any VDA5050-conformant fleet manager can drive this simulator directly. A few real, connectable options if you don't have your own:

- **[openTCS](https://github.com/openTCS/opentcs)** (Fraunhofer IML) — the most complete open-source fleet management system with a real VDA5050 integration via its [`opentcs-commadapter-vda5050`](https://github.com/openTCS/opentcs-commadapter-vda5050) plugin. Point its comm adapter at the same MQTT broker and it'll drive these robots like real AGVs.
- **[NVIDIA Isaac Mission Dispatch](https://github.com/nvidia-isaac/isaac_mission_dispatch)** — a maintained, VDA5050-compatible mission-dispatch service if you want something closer to production-grade fleet-manager behavior than a toy.
- **[vda-5050-lib.js](https://github.com/coatyio/vda-5050-lib.js)** — if you'd rather script your own master control quickly (TypeScript/Node, ships a `MasterController` class and JSON-schema-validated messages).

## Simulating older VDA5050 protocol versions

Every robot in `fleet.default.yaml` can set `protocol_version` (default `"3.0.0"`). Two entries do by default — `go2-legacy` (`2.1.0`) and `pidog-legacy` (`1.1.0`) — so a mixed-version fleet is there out of the box, no config changes needed:

```yaml
robots:
  - id: go2-legacy
    model: go2
    protocol_version: "2.1.0"
```

This isn't just a different `version` string on an otherwise-identical v3.0.0 payload — the internal `SimulatedAgv` engine (movement, battery, order/action state machine) is exactly the same for every robot regardless of version, but a legacy-configured robot differs from a v3.0.0 one in two real ways:

1. **Wire shape.** State/connection payloads are downgraded at the transport boundary (`src/vda5050_sim/legacy_shapes.py`) to the field names 1.1.0/2.0.0/2.1.0 actually use — these three are structurally identical to each other, and only differ from 3.0.0 at a small, well-defined set of fields:

   | v3.0.0 | pre-3.0 (1.1.0 / 2.0.0 / 2.1.0) |
   |---|---|
   | `mobileRobotPosition` | `agvPosition` (no `localizationScore`/`deviationRange`; has `positionInitialized`) |
   | `powerSupply.stateOfCharge` / `.range` | `batteryState.batteryCharge` / `.reach` (no `batteryCurrent`) |
   | `safetyState.activeEmergencyStop` | `safetyState.eStop` |
   | `connectionState: CONNECTION_BROKEN` | `connectionState: CONNECTIONBROKEN` |
   | (`HIBERNATING`) | mapped to `OFFLINE` — no legacy hibernation flow is simulated |

   A legacy robot also publishes on its own version-scoped subject/topic: NATS `vda5050.v1`/`vda5050.v2` (instead of the default `vda5050.v3`), MQTT `vda5050/v1`/`vda5050/v2`.

2. **Capability gate.** 3.0.0-only functionality — confirmed against the VDA5050 project's own 3.0.0 release notes — is genuinely absent, not just relabeled: zones (the `zoneSet` topic isn't subscribed at all, and `downloadZoneSet`/`enableZoneSet`/`deleteZoneSet` are rejected), `updateCertificate` and `trigger` instant actions are rejected, and a fault-injected retriable-action failure goes straight to `FAILED` instead of the 3.0.0-only `RETRIABLE` state. Rejections use the spec-defined `invalidInstantAction` error (`src/vda5050_sim/agv.py`'s `LEGACY_UNSUPPORTED_ACTIONS`).

**Known scope limit**: only *outgoing* messages (state/connection/factsheet/visualization) are version-shaped. Incoming orders/instant actions are still parsed as v3.0.0 regardless of a robot's `protocol_version` — legacy robots exist to be *discovered and read* correctly by a fleet manager across versions, not to receive genuinely legacy-shaped (e.g. `startNodeId`/`endNodeId`-addressed) orders.

## Architecture and Runtime

- FastAPI service (Python 3.11, `uv`), one `SimulatedAgv` state machine + async pub/sub task set per configured robot (`src/vda5050_sim/agv.py`, `fleet.py`).
- A `Transport` abstraction (`src/vda5050_sim/transport.py`) is the only thing that differs between the two modes. `NatsTransport`: one shared connection, dot-separated subjects, no retain/LWT. `MqttTransport`: **one `aiomqtt.Client` per robot**, slash-separated topics, real `retain`/Last-Will-Testament support. `Fleet` obtains a `Transport` per robot via `build_transport_factory()` — for NATS that returns the same connected shared instance every time; for MQTT it connects a fresh per-robot client (with its own Will) each time. `RobotRuntime`/`SimulatedAgv` logic is identical either way.
- Wire schemas (`src/vda5050_sim/schemas.py`) are implemented directly against the official [VDA5050 3.0.0 JSON Schemas](https://github.com/VDA5050/VDA5050) — including field names that differ from common v1/v2-era naming (e.g. `nodeDescriptor` not `nodeDescription`, `maximumSpeed` not `maxSpeed`), and the fact that v3.0.0 edges have no `startNodeId`/`endNodeId` at all (traversal order comes purely from a shared node/edge `sequenceId` space).
- Action progress (node-bound, edge-bound, and generic instant actions alike) is advanced through one shared state machine keyed off an `_action_registry` that survives a node/edge being dropped from the remaining-graph list — otherwise a non-blocking action started on a since-departed node/edge would freeze at `RUNNING` forever.
- Traffic control (`Edge.corridor.releaseRequired` -> `EdgeRequest`, `RELEASE`/`COORDINATED_REPLANNING` zone membership via point-in-polygon -> `ZoneRequest`) gates movement exactly like the existing `released` node/edge gate, resolved by a `responses` message from fleet control.

## API Surface

- **MQTT** (subscribe, per robot): `.../order`, `.../instantActions`, `.../zoneSet`, `.../responses`. (publish, retained except visualization): `.../state`, `.../visualization`, `.../connection`, `.../factsheet`.
- **NATS**: same message types, dot-separated subjects (`vda5050.v3.{manufacturer}.{serial}.{type}` by default — `v1`/`v2` for robots configured with an older `protocol_version`, see [Simulating older VDA5050 protocol versions](#simulating-older-vda5050-protocol-versions)), no retain.
- **REST**: `GET /health` (liveness), `GET /fleet` (JSON snapshot of every robot's live state), `GET /logs?n=200` (recent structured event log), `GET /` (log-viewer UI).

## Configuration

| Env var | Default | Purpose |
|---|---|---|
| `TRANSPORT` | `nats` | `mqtt` (standalone, one connection per robot) or `nats` |
| `MQTT_HOST` | `localhost` | MQTT broker host (transport=mqtt) |
| `MQTT_PORT` | `1883` | MQTT broker port |
| `MQTT_USERNAME` / `MQTT_PASSWORD` | unset | MQTT auth, if the broker requires it |
| `MQTT_TLS` | `false` | Enable TLS to the MQTT broker |
| `MQTT_TOPIC_PREFIX` | `vda5050/v3` | MQTT topic prefix |
| `NATS_BROKER` (or `NATS_URL`) | `nats://localhost:4222` | NATS server (transport=nats) |
| `VDA5050_PREFIX` | `vda5050.v3` | NATS subject prefix |
| `FLEET_CONFIG_PATH` | `fleet.default.yaml` | Robot roster (see file for schema) |
| `STATE_HZ` | `1.0` | `state` publish rate |
| `VISUALIZATION_HZ` | `2.0` | `visualization` publish rate |
| `CONNECTION_HEARTBEAT_S` | `2.0` | `connection` heartbeat interval |
| `TICK_S` | `0.2` | Movement/order-processing tick |
| `ACTION_DURATION_S` | `1.0` | Simulated duration of a node/edge/instant action |
| `DEFAULT_SPEED_MPS` | `0.5` | Fallback linear speed if a robot has no `max_speed` |
| `DEFAULT_ANGULAR_SPEED_RAD_S` | `1.0` | Fallback angular speed if a robot has no `angular_speed` |
| `HORIZON_THRESHOLD_NODES` | `2` | Proactively set `newBaseRequest` once this many released nodes remain |
| `DEFAULT_BATTERY_DRAIN_PERCENT_PER_METER` | `0.05` | Battery drain while driving, if a robot has no override |
| `DEFAULT_BATTERY_CHARGE_PERCENT_PER_S` | `5.0` | Battery charge rate while `charging`, if a robot has no override |
| `PORT` | `8000` | HTTP port |

Per-robot overrides (`max_speed`, `angular_speed`, `battery_drain_percent_per_meter`, `battery_charge_percent_per_s`, `protocol_version`, `fault_profile.{connection_drop,error_injection,field_violation,service_mode,emergency_stop}_probability`) live in `fleet.default.yaml` — see that file for the full per-robot schema.

## Local Development

```bash
uv sync --extra dev

# Standalone/MQTT:
mosquitto &
TRANSPORT=mqtt uv run uvicorn vda5050_sim.main:app --reload --port 8000

# NATS:
nats-server &
uv run uvicorn vda5050_sim.main:app --reload --port 8000

uv run pytest      # conformance suite (spins up its own nats-server/mosquitto)
uv run ruff check
```

## Known simplifications

- **No trajectory (NURBS) path following.** Movement is straight-line node-to-node plus per-edge speed/orientation overrides — covers arrival timing, blocking, and orientation (what fleet-manager conformance testing actually exercises), not the exact path shape. A large, separable addition if you need it. `state.plannedPath`/`intermediatePath` stay unset for the same reason.
- **3 of VDA5050's 10 zone types are accepted/stored only, with no simulated runtime effect: `PRIORITY`, `PENALTY`, `LINE_GUIDED`.** They only mean something with a real path planner or cost function to influence, which this simulator deliberately doesn't have. The other 7 (`RELEASE`/`COORDINATED_REPLANNING`/`BLOCKED`/`SPEED_LIMIT`/`ACTION`/`DIRECTED`/`BIDIRECTED`) all have real effects — see above.
- **`downloadMap`/`downloadZoneSet` don't actually fetch anything.** Both carry a `*DownloadLink` parameter per the spec, but this simulator only simulates the timed download *lifecycle* (`WAITING`→`RUNNING`→`FINISHED`, added to `state.maps`/tracked zone sets) — no HTTP fetch happens. Same treatment for `updateCertificate` (simulated lifecycle, no real TLS cert install).
- **MQTT Last-Will-Testament payload uses a fixed sentinel `headerId` and a connect-time (not actual-disconnect-time) timestamp** — a will payload is captured once at connect and can't be dynamically updated. Only `connectionState: CONNECTION_BROKEN` matters for a fleet manager's crash-detection purposes.
- **`trigger`'s correlation to a specific `waitForTrigger` action is a judgment call.** Table 4 lists an `actionId` parameter for `retry`/`skipRetry` but not for `trigger` — the spec doesn't fully define how `trigger` picks a target when several `waitForTrigger` actions are outstanding at once. This simulator honors `actionId` if the sender supplies one anyway (a practical superset), otherwise releases the oldest still-`RUNNING` `waitForTrigger`.
- **No physical charger-dock modeling.** `startCharging`/`stopCharging` just toggle a charge-rate timer wherever/whenever commanded.
- **`blockingTypes`/`pauseAllowed`/`cancelAllowed` in the factsheet are this simulator's own reasonable declarations, not spec-mandated values** — VDA5050 leaves those as each manufacturer's own capability self-declaration for every actionType. See `action_catalog.py`'s module docstring for the reasoning (grounded in Table 5's per-action state semantics where possible).

Two genuine upstream spec bugs were found and fixed when vendoring the official JSON Schemas — see `src/vda5050_sim/json_schemas/README.md`. Two more (a self-contradictory `connectionState` enum, a `typeSpecification` required-field typo) are documented rather than silently patched — see `tests/test_schema_validation.py`. A third, in the raw prose of Table 4 itself (the `startHibernation`/`stopHibernation` rows are missing their trailing `zone` scope column), is documented in `action_catalog.py`.

## Troubleshooting

- **A fleet manager can't see the robots over MQTT**: confirm `TRANSPORT=mqtt` and that both sides point at the same broker/`MQTT_TOPIC_PREFIX`; `mosquitto_sub -t 'vda5050/v3/#' -v` is the fastest way to check traffic is flowing at all.
- **A killed/crashed robot isn't reported as `CONNECTION_BROKEN`**: confirm you're on `TRANSPORT=mqtt` (NATS mode has no LWT equivalent) and that the broker actually saw an ungraceful disconnect (a clean shutdown publishes `OFFLINE` instead, by design).

## Related

- [VDA5050 specification](https://github.com/VDA5050/VDA5050) — this simulator's schemas (`src/vda5050_sim/schemas.py`) and vendored JSON Schemas (`src/vda5050_sim/json_schemas/`) are implemented directly against it.
