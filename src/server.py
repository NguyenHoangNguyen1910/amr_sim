"""Serve unified manual and autonomous AMR control over Gazebo Transport."""

from __future__ import annotations

import argparse
import json
import logging
import os

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from config import CONFIG
from mapping_api import mapping_router
from navigation.navigator import (
    NavigationConflict,
    Navigator,
)
from navigation.global_planner import PlanningError
from navigation.types import Point2D
from navigation.helpers import SensorDataError
from transport import AMRTransport


LOGGER = logging.getLogger("uvicorn.error")


class VelocityCommand(BaseModel):
    """Validated manual differential-drive velocity command."""

    model_config = ConfigDict(extra="forbid")

    linear: float = Field(strict=True)
    angular: float = Field(strict=True)


class GoalPoint(BaseModel):
    model_config = ConfigDict(extra="forbid")

    x: float = Field(strict=True)
    y: float = Field(strict=True)


class NavigationPlanRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    goal: GoalPoint


class SpeedRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    speed_mps: float = Field(strict=True)


class RouteRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    waypoints: list[GoalPoint] = Field(min_length=1, max_length=100)


class WallsRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    walls: list[tuple[GoalPoint, GoalPoint]] = Field(max_length=100)


def create_app(
    transport: AMRTransport,
    navigator: Navigator,
) -> FastAPI:
    """Build the AMR HTTP API and mount self-contained viewer assets."""

    if not CONFIG.paths.visualize_root.is_dir():
        raise RuntimeError(
            f"Visualization assets not found: {CONFIG.paths.visualize_root}"
        )
    if not CONFIG.paths.models_root.is_dir():
        raise RuntimeError(f"Model assets not found: {CONFIG.paths.models_root}")

    app = FastAPI(
        title="AMR Control",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
    )
    app.include_router(mapping_router(
        lambda after: transport.mapping_sample(after),
        os.environ.get("AMR_SLAM_KEY", ""),
    ))

    @app.get("/api/health")
    def health():
        telemetry = transport.telemetry_snapshot()
        return {
            "ok": True,
            "ready": telemetry["ready"],
            "world": CONFIG.simulation.world_name,
            "model": CONFIG.simulation.model_name,
            "websocket_port": CONFIG.server.websocket_port,
            "partition": os.environ.get("GZ_PARTITION", ""),
            "freshness": telemetry["freshness"],
            "navigation": navigator.status(),
        }

    @app.get("/api/amr/telemetry")
    def telemetry():
        return transport.telemetry_snapshot()

    @app.get("/api/sensors")
    def sensor_profile():
        return transport.profile_snapshot()

    @app.get("/api/sensors/depth")
    def depth_frame():
        frame = transport.depth_frame()
        if frame is None:
            return JSONResponse(status_code=503, content={"error": "Depth frame unavailable"})
        info, pixels = frame
        return Response(
            pixels, media_type="application/octet-stream",
            headers={
                "Cache-Control": "no-store",
                "X-Depth-Width": str(info["width"]),
                "X-Depth-Height": str(info["height"]),
                "X-Depth-Step": str(info["step"]),
                "X-Depth-Sequence": str(info["sequence"]),
            },
        )

    @app.post("/api/amr/command")
    def command(command: VelocityCommand):
        if navigator.owns_control:
            navigator.cancel()
            LOGGER.info("Manual command cancelled the active navigation mission")
        try:
            applied = transport.set_velocity(command.linear, command.angular)
        except ValueError as error:
            LOGGER.warning("Rejected AMR command: %s", error)
            return JSONResponse(
                status_code=400, content={"ok": False, "error": str(error)}
            )
        except RuntimeError as error:
            LOGGER.warning("AMR command unavailable: %s", error)
            return JSONResponse(
                status_code=409, content={"ok": False, "error": str(error)}
            )
        LOGGER.info(
            "AMR command published: linear=%.3f m/s angular=%.3f rad/s",
            applied["linear"],
            applied["angular"],
        )
        return {"ok": True, **applied}

    @app.post("/api/amr/speed")
    def speed(request: SpeedRequest):
        """Set the shared browser/DWA speed cap."""

        try:
            status = navigator.set_speed_mps(request.speed_mps)
        except ValueError as error:
            return JSONResponse(
                status_code=422,
                content={"ok": False, "code": "INVALID_SPEED", "error": str(error)},
            )
        return {"ok": True, "speed_mps": status["speed_mps"]}

    @app.post("/api/amr/reset")
    def reset():
        try:
            status = navigator.reset()
        except NavigationConflict as error:
            return JSONResponse(
                status_code=409, content={"ok": False, "error": str(error)}
            )
        except RuntimeError as error:
            LOGGER.warning("AMR reset failed: %s", error)
            return JSONResponse(
                status_code=503, content={"ok": False, "error": str(error)}
            )
        ok = status["error"] is None
        LOGGER.info("AMR reset %s", "completed" if ok else "failed")
        return JSONResponse(
            status_code=200 if ok else 503,
            content={"ok": ok, **status},
        )

    def navigation_error(error: Exception) -> JSONResponse:
        if isinstance(error, PlanningError):
            status_code = 422
            code = error.code
        elif isinstance(error, SensorDataError):
            status_code = 503
            code = "SENSORS_NOT_READY"
        elif isinstance(error, NavigationConflict):
            status_code = 409
            code = "STATE_CONFLICT"
        elif isinstance(error, ValueError):
            status_code = 422
            code = "INVALID_GOAL"
        else:
            status_code = 500
            code = "NAVIGATION_ERROR"
        LOGGER.warning("Navigation request failed: %s", error)
        return JSONResponse(
            status_code=status_code,
            content={"ok": False, "code": code, "error": str(error)},
        )

    @app.post("/api/navigation/plan")
    def navigation_plan(request: NavigationPlanRequest):
        try:
            status = navigator.plan(
                Point2D(request.goal.x, request.goal.y)
            )
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    layout = json.loads(CONFIG.paths.navigation_map_path.read_text(encoding="utf-8"))

    @app.get("/api/map")
    def map_layout():
        return layout

    @app.get("/api/map/live")
    def map_live():
        return {"pose": transport.model_pose_snapshot(), "navigation": navigator.status()}

    @app.post("/api/navigation/route")
    def navigation_route(request: RouteRequest):
        try:
            status = navigator.plan_route(tuple(Point2D(p.x, p.y) for p in request.waypoints))
        except (PlanningError, SensorDataError, NavigationConflict, ValueError) as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.get("/api/map/walls")
    def map_walls():
        return {"walls": [[p.to_dict() for p in wall] for wall in navigator.virtual_walls()]}

    @app.put("/api/map/walls")
    def update_map_walls(request: WallsRequest):
        try:
            navigator.set_virtual_walls(
                tuple((Point2D(a.x, a.y), Point2D(b.x, b.y)) for a, b in request.walls)
            )
        except (NavigationConflict, ValueError) as error:
            return navigation_error(error)
        return map_walls()

    @app.post("/api/navigation/validate-goal")
    def navigation_validate_goal(request: NavigationPlanRequest):
        try:
            valid = navigator.is_goal_free(Point2D(request.goal.x, request.goal.y))
        except Exception as error:
            return navigation_error(error)
        return {
            "ok": True,
            "valid": valid,
            "code": None if valid else "GOAL_BLOCKED",
            "message": (
                "Goal is valid" if valid else "Goal is blocked or outside the map"
            ),
        }

    @app.post("/api/navigation/start")
    def navigation_start():
        try:
            status = navigator.start()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/cancel")
    def navigation_cancel():
        try:
            status = navigator.cancel()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.get("/api/navigation/status")
    def navigation_status():
        try:
            status = navigator.status()
        except Exception as error:
            return navigation_error(error)
        return status

    app.mount(
        "/assets/models",
        StaticFiles(directory=CONFIG.paths.models_root),
        name="models",
    )
    app.mount(
        "/",
        StaticFiles(directory=CONFIG.paths.visualize_root, html=True),
        name="visualize",
    )
    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=CONFIG.server.host)
    parser.add_argument("--port", type=int, default=CONFIG.server.http_port)
    parser.add_argument("--partition", default=CONFIG.server.partition)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    os.environ.setdefault("GZ_PARTITION", args.partition)
    transport = AMRTransport()
    navigator = Navigator(transport)
    try:
        print(
            f"Serving {CONFIG.paths.visualize_root} on "
            f"http://{args.host}:{args.port}; publishing "
            f"{CONFIG.simulation.command_topic} "
            f"using GZ_PARTITION={os.environ['GZ_PARTITION']}"
        )
        uvicorn.run(
            create_app(transport, navigator),
            host=args.host,
            port=args.port,
            access_log=False,
        )
    except KeyboardInterrupt:
        print("Stopping AMR control bridge")
    finally:
        navigator.close()
        transport.close()


if __name__ == "__main__":
    main()
