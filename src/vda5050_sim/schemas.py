"""VDA5050 v3.0.0 message schemas, implemented directly against the official
spec's JSON Schemas (github.com/VDA5050/VDA5050, tag 3.0.0) rather than any
third-party reimplementation — including the real v3.0.0 field names, which
differ from common v1/v2-era naming in several places (e.g. `nodeDescriptor`
not `nodeDescription`, `maximumSpeed` not `maxSpeed`, and edges identified by
`sequenceId` rather than explicit `startNodeId`/`endNodeId`).

See `src/vda5050_sim/json_schemas/` for the vendored official schemas these
models are validated against in tests/test_schema_validation.py.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any

from pydantic import BaseModel, Field

# ── Enums (values verified against the official 3.0.0 JSON Schemas) ─────────


class OperatingMode(StrEnum):
    STARTUP = "STARTUP"
    AUTOMATIC = "AUTOMATIC"
    SEMIAUTOMATIC = "SEMIAUTOMATIC"
    INTERVENED = "INTERVENED"
    MANUAL = "MANUAL"
    SERVICE = "SERVICE"
    TEACH_IN = "TEACH_IN"


class ActionStatus(StrEnum):
    WAITING = "WAITING"
    INITIALIZING = "INITIALIZING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    RETRIABLE = "RETRIABLE"
    FINISHED = "FINISHED"
    FAILED = "FAILED"


class ErrorLevel(StrEnum):
    WARNING = "WARNING"
    URGENT = "URGENT"
    CRITICAL = "CRITICAL"
    FATAL = "FATAL"


class InfoLevel(StrEnum):
    INFO = "INFO"
    DEBUG = "DEBUG"


class BlockingType(StrEnum):
    NONE = "NONE"
    SOFT = "SOFT"
    SINGLE = "SINGLE"
    HARD = "HARD"


class ConnectionState(StrEnum):
    ONLINE = "ONLINE"
    OFFLINE = "OFFLINE"
    HIBERNATING = "HIBERNATING"
    # The official schema's enum uses this literal (with underscore), even
    # though that same schema file's own prose describes it without one
    # ("CONNECTIONBROKEN") — a known inconsistency in the upstream VDA5050
    # 3.0.0 spec release. We follow the enum, since that's what strict
    # schema validation (and most real implementations) actually check.
    CONNECTION_BROKEN = "CONNECTION_BROKEN"


class EmergencyStopType(StrEnum):
    """safetyState.activeEmergencyStop — the only e-stop field in v3.0.0
    (v1/v2's separate `eStop`/`AUTOACK` do not exist in the real v3 schema)."""

    MANUAL = "MANUAL"
    REMOTE = "REMOTE"
    NONE = "NONE"


class OrientationType(StrEnum):
    GLOBAL = "GLOBAL"
    TANGENTIAL = "TANGENTIAL"


class ActionScope(StrEnum):
    INSTANT = "INSTANT"
    NODE = "NODE"
    EDGE = "EDGE"
    ZONE = "ZONE"


# ── Shared sub-models ────────────────────────────────────────────────────────


class ActionParameter(BaseModel):
    key: str
    value: Any


class Action(BaseModel):
    actionId: str
    actionType: str
    blockingType: BlockingType = BlockingType.NONE
    actionDescriptor: str | None = None
    actionParameters: list[ActionParameter] = Field(default_factory=list)
    retriable: bool | None = None


class NodePosition(BaseModel):
    x: float
    y: float
    mapId: str = "default"
    theta: float | None = None
    allowedDeviationXY: float | None = None
    allowedDeviationTheta: float | None = None


class Node(BaseModel):
    """Order node. `sequenceId` is shared with `Edge` and is the only thing
    that defines traversal order in v3.0.0 — there is no separate edge
    start/end node reference."""

    nodeId: str
    sequenceId: int
    released: bool = True
    nodeDescriptor: str | None = None
    nodePosition: NodePosition | None = None
    actions: list[Action] = Field(default_factory=list)


class Edge(BaseModel):
    edgeId: str
    sequenceId: int
    released: bool = True
    edgeDescriptor: str | None = None
    maximumSpeed: float | None = None
    maximumMobileRobotHeight: float | None = None
    minimumLoadHandlingDeviceHeight: float | None = None
    orientation: float | None = None
    orientationType: OrientationType | None = None
    direction: str | None = None
    reachOrientationBeforeEntering: bool | None = None
    maxRotationSpeed: float | None = None
    trajectory: Any | None = None
    length: float | None = None
    corridor: Any | None = None
    actions: list[Action] = Field(default_factory=list)


class Velocity(BaseModel):
    vx: float | None = None
    vy: float | None = None
    omega: float | None = None


class MobileRobotPosition(BaseModel):
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    mapId: str = "default"
    localized: bool = True
    localizationScore: float | None = Field(default=None, ge=0.0, le=1.0)
    deviationRange: float | None = None


class PowerSupply(BaseModel):
    stateOfCharge: float = Field(default=0.0, ge=0, le=100)
    charging: bool = False
    batteryVoltage: float | None = None
    batteryCurrent: float | None = None
    batteryHealth: float | None = Field(default=None, ge=0, le=100)
    range: float | None = None


class SafetyState(BaseModel):
    activeEmergencyStop: EmergencyStopType = EmergencyStopType.NONE
    fieldViolation: bool = False


class ErrorReference(BaseModel):
    referenceKey: str
    referenceValue: str


class Error(BaseModel):
    errorType: str
    errorLevel: ErrorLevel
    errorReferences: list[ErrorReference] = Field(default_factory=list)
    errorDescription: str | None = None
    errorHint: str | None = None


class Information(BaseModel):
    infoType: str
    infoLevel: InfoLevel = InfoLevel.INFO
    infoReferences: list[ErrorReference] = Field(default_factory=list)
    infoDescriptor: str | None = None


class Load(BaseModel):
    loadId: str | None = None
    loadType: str | None = None
    loadPosition: str | None = None
    weight: float | None = None


class NodeState(BaseModel):
    nodeId: str
    sequenceId: int
    released: bool = True
    nodeDescriptor: str | None = None
    nodePosition: NodePosition | None = None


class EdgeState(BaseModel):
    edgeId: str
    sequenceId: int
    released: bool = True
    edgeDescriptor: str | None = None
    trajectory: Any | None = None


class ActionState(BaseModel):
    actionId: str
    actionStatus: ActionStatus
    actionType: str | None = None
    actionDescriptor: str | None = None
    actionResult: str | None = None


# ── Top-level messages ───────────────────────────────────────────────────────


class OrderMessage(BaseModel):
    headerId: int
    timestamp: str
    version: str = "3.0.0"
    manufacturer: str
    serialNumber: str
    orderId: str
    orderUpdateId: int = 0
    orderDescription: str | None = None
    nodes: list[Node] = Field(default_factory=list)
    edges: list[Edge] = Field(default_factory=list)


class InstantActionsMessage(BaseModel):
    headerId: int
    timestamp: str
    version: str = "3.0.0"
    manufacturer: str
    serialNumber: str
    actions: list[Action] = Field(default_factory=list)


class ConnectionMessage(BaseModel):
    headerId: int
    timestamp: str
    version: str = "3.0.0"
    manufacturer: str
    serialNumber: str
    connectionState: ConnectionState


class StateMessage(BaseModel):
    headerId: int
    timestamp: str
    version: str = "3.0.0"
    manufacturer: str
    serialNumber: str
    orderId: str = ""
    orderUpdateId: int = 0
    lastNodeId: str = ""
    lastNodeSequenceId: int = 0
    nodeStates: list[NodeState] = Field(default_factory=list)
    edgeStates: list[EdgeState] = Field(default_factory=list)
    driving: bool = False
    paused: bool = False
    newBaseRequest: bool = False
    mobileRobotPosition: MobileRobotPosition | None = None
    velocity: Velocity | None = None
    powerSupply: PowerSupply = Field(default_factory=PowerSupply)
    operatingMode: OperatingMode = OperatingMode.AUTOMATIC
    actionStates: list[ActionState] = Field(default_factory=list)
    instantActionStates: list[ActionState] = Field(default_factory=list)
    zoneActionStates: list[ActionState] = Field(default_factory=list)
    errors: list[Error] = Field(default_factory=list)
    information: list[Information] = Field(default_factory=list)
    loads: list[Load] = Field(default_factory=list)
    safetyState: SafetyState = Field(default_factory=SafetyState)
    distanceSinceLastNode: float | None = None
    # Present in the schema but unused by this simulator (no zone/map
    # authoring support) — always empty/omitted via exclude_none.
    maps: list[Any] = Field(default_factory=list)
    zoneSets: list[Any] = Field(default_factory=list)
    plannedPath: Any | None = None
    intermediatePath: Any | None = None
    zoneRequests: list[Any] = Field(default_factory=list)
    edgeRequests: list[Any] = Field(default_factory=list)


class VisualizationMessage(BaseModel):
    headerId: int
    timestamp: str
    version: str = "3.0.0"
    manufacturer: str
    serialNumber: str
    referenceStateHeaderId: int
    mobileRobotPosition: MobileRobotPosition | None = None
    velocity: Velocity | None = None
    plannedPath: Any | None = None
    intermediatePath: Any | None = None


# ── Factsheet ────────────────────────────────────────────────────────────────


class TypeSpecification(BaseModel):
    seriesName: str
    mobileRobotClass: str
    maximumLoadMass: float
    localizationTypes: list[str]
    navigationTypes: list[str]
    seriesDescription: str | None = None
    mobileRobotKinematics: str | None = None


class PhysicalParameters(BaseModel):
    minimumSpeed: float
    maximumSpeed: float
    maximumAcceleration: float
    maximumDeceleration: float
    minimumHeight: float
    maximumHeight: float
    width: float
    length: float
    minimumAngularSpeed: float | None = None
    maximumAngularSpeed: float | None = None


class MaximumStringLengths(BaseModel):
    maximumIdLength: int | None = None
    maximumTopicSerialLength: int | None = None
    maximumTopicElementLength: int | None = None


class MaximumArrayLengths(BaseModel):
    instantActions: int | None = None


class Timing(BaseModel):
    minimumOrderInterval: float
    minimumStateInterval: float
    defaultStateInterval: float | None = None
    visualizationInterval: float | None = None


class ProtocolLimits(BaseModel):
    maximumStringLengths: MaximumStringLengths = Field(default_factory=MaximumStringLengths)
    maximumArrayLengths: MaximumArrayLengths = Field(default_factory=MaximumArrayLengths)
    timing: Timing


class ActionParameterDefinition(BaseModel):
    key: str
    valueDataType: str
    description: str | None = None
    isOptional: bool | None = None


class MobileRobotActionDef(BaseModel):
    actionType: str
    actionScopes: list[ActionScope]
    pauseAllowed: str = "false"
    cancelAllowed: str = "false"
    actionDescription: str | None = None
    actionParameters: list[ActionParameterDefinition] = Field(default_factory=list)
    blockingTypes: list[BlockingType] = Field(default_factory=list)


class OptionalParameter(BaseModel):
    parameter: str
    support: str
    description: str | None = None


class ProtocolFeatures(BaseModel):
    mobileRobotActions: list[MobileRobotActionDef] = Field(default_factory=list)
    optionalParameters: list[OptionalParameter] = Field(default_factory=list)


class Vertex2d(BaseModel):
    x: float
    y: float


class Envelope2d(BaseModel):
    envelope2dId: str
    vertices: list[Vertex2d]
    description: str | None = None


class MobileRobotGeometry(BaseModel):
    envelopes2d: list[Envelope2d] = Field(default_factory=list)


class LoadSpecification(BaseModel):
    loadPositions: list[Any] = Field(default_factory=list)
    loadSets: list[Any] = Field(default_factory=list)


class FactsheetMessage(BaseModel):
    headerId: int
    timestamp: str
    version: str = "3.0.0"
    manufacturer: str
    serialNumber: str
    typeSpecification: TypeSpecification
    physicalParameters: PhysicalParameters
    protocolLimits: ProtocolLimits
    protocolFeatures: ProtocolFeatures
    mobileRobotGeometry: MobileRobotGeometry = Field(default_factory=MobileRobotGeometry)
    loadSpecification: LoadSpecification = Field(default_factory=LoadSpecification)
