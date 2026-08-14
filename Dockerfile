# syntax=docker/dockerfile:1.7
#
# vda5050-sim runtime image — VDA5050 fleet simulator + log-viewer UI.
#
FROM python:3.11-slim AS runtime
WORKDIR /app

COPY --from=ghcr.io/astral-sh/uv:0.5.11 /uv /usr/local/bin/uv

ENV UV_LINK_MODE=copy \
    UV_COMPILE_BYTECODE=1 \
    UV_NO_PROGRESS=1

COPY pyproject.toml README.md ./
COPY src/ ./src/
COPY fleet.default.yaml ./
RUN uv sync --no-dev

COPY ui/ ./ui/

ENV PYTHONUNBUFFERED=1 \
    PATH="/app/.venv/bin:${PATH}"

EXPOSE 8000

CMD ["uvicorn", "vda5050_sim.main:app", "--host", "0.0.0.0", "--port", "8000"]
