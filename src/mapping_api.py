"""Session and occupancy grid exchange with the Cartographer sidecar."""

from __future__ import annotations

import secrets
import threading
import time
import uuid

from fastapi import APIRouter, Body, Header, HTTPException
from pydantic import BaseModel, Field


class Grid(BaseModel):
    width: int = Field(ge=1, le=2048)
    height: int = Field(ge=1, le=2048)
    resolution: float = Field(gt=0, le=1)
    x: float = Field(allow_inf_nan=False)
    y: float = Field(allow_inf_nan=False)
    cells: list[int] = Field(max_length=1_000_000)


def mapping_router(sample, token: str) -> APIRouter:
    router = APIRouter(prefix="/api/slam")
    lock = threading.Lock()
    session = None
    active = False
    grid = None
    revision = 0
    last_bridge = float("-inf")

    def authorize(key: str) -> None:
        if not token or not secrets.compare_digest(key.encode(), token.encode()):
            raise HTTPException(403, "Mapping bridge authentication failed")

    @router.post("/start")
    def start():
        nonlocal session, active, grid, revision
        with lock:
            if time.monotonic() - last_bridge > 5:
                raise HTTPException(503, "Cartographer bridge is offline")
            session = uuid.uuid4().hex
            active, grid, revision = True, None, 0
            return {"session": session, "active": active, "revision": revision}

    @router.post("/stop")
    def stop():
        nonlocal active
        with lock:
            active = False
            return {"session": session, "active": active, "revision": revision}

    @router.get("/map")
    def get_map(after: int = -1, previous: str = ""):
        with lock:
            return {
                "session": session,
                "active": active,
                "revision": revision,
                "online": time.monotonic() - last_bridge <= 5,
                "grid": grid if previous != session or after != revision else None,
            }

    @router.get("/bridge")
    def bridge(after: int = 0, previous: str = "", x_slam_key: str = Header("")):
        nonlocal last_bridge
        authorize(x_slam_key)
        with lock:
            last_bridge = time.monotonic()
            current = session if active else None
        data = sample(after) if current and current == previous else {}
        return {"session": current, **data}

    @router.put("/bridge/{source}")
    def update(source: str, data: Grid = Body(), x_slam_key: str = Header("")):
        nonlocal grid, revision
        authorize(x_slam_key)
        if len(data.cells) != data.width * data.height or any(
            not -1 <= cell <= 100 for cell in data.cells
        ):
            raise HTTPException(422, "Invalid occupancy grid")
        new_grid = data.model_dump()
        with lock:
            if active and source == session:
                grid = new_grid
                revision += 1
        return {"ok": True}

    return router
