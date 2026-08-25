from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse

from vda5050_sim.config import get_settings
from vda5050_sim.fleet import Fleet, load_fleet_config
from vda5050_sim.logbuffer import LogBuffer
from vda5050_sim.transport import build_transport_factory

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("vda5050_sim")

settings = get_settings()
log_buffer = LogBuffer(maxlen=settings.log_buffer_size)

_UI_DIR = Path(__file__).resolve().parent.parent.parent / "ui"
_UI_INDEX = _UI_DIR / "index.html"
_APP_ICON = _UI_DIR / "app_icon.svg"


@asynccontextmanager
async def lifespan(app: FastAPI):
    transport_factory = await build_transport_factory(settings)
    configs = load_fleet_config(settings.fleet_config_path)
    fleet = Fleet(settings, transport_factory, log_buffer)
    await fleet.start(configs)
    app.state.fleet = fleet
    if settings.transport == "mqtt":
        logger.info(
            "vda5050-sim started with %d robots over MQTT %s:%d (prefix=%s)",
            len(configs),
            settings.mqtt_host,
            settings.mqtt_port,
            settings.mqtt_topic_prefix,
        )
    else:
        logger.info(
            "vda5050-sim started with %d robots on NATS %s (prefix=%s)",
            len(configs),
            settings.nats_broker,
            settings.vda5050_prefix,
        )
    try:
        yield
    finally:
        await fleet.stop()  # closes every unique Transport it holds


app = FastAPI(title="vda5050-sim", lifespan=lifespan)


@app.get("/health")
async def health() -> dict:
    return {"status": "ok"}


@app.get("/fleet")
async def fleet_snapshot() -> JSONResponse:
    return JSONResponse(app.state.fleet.snapshot())


@app.get("/logs")
async def logs(n: int = 200) -> JSONResponse:
    return JSONResponse(log_buffer.tail(n))


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(_UI_INDEX)


@app.get("/app_icon.svg")
async def app_icon() -> FileResponse:
    return FileResponse(_APP_ICON, media_type="image/svg+xml")
