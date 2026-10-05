"""On-demand yaw-aligned terrain heights, without a simulated lidar."""

from dataclasses import dataclass, field
import colorsys

import mujoco
import numpy as np


@dataclass(frozen=True)
class TerrainScan:
    """Arrays indexed [forward_index, lateral_index]; misses are NaN / -1.

    x and y are N x M arrays of world coordinates for each ray origin.
    """

    time: float
    center: np.ndarray
    x: np.ndarray
    y: np.ndarray
    ray_start_height: float
    heights: np.ndarray
    geom_ids: np.ndarray
    # Body-to-world rotation captured with the scan, including roll and pitch.
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))

    @property
    def valid(self) -> np.ndarray:
        return self.geom_ids >= 0

    def add_visuals(self, scene: mujoco.MjvScene, radius: float = 0.025,
                    height_range: tuple[float, float] | None = None) -> None:
        """Append spheres at valid hits without clearing existing scene geoms.

        Hue runs from blue (low) through green to red (high). By default the
        color scale spans this scan's minimum and maximum world heights; a flat
        scan is green. Pass a fixed height_range for consistent colors across
        scans. Heights outside that range are clamped. Misses are omitted.
        Call under the viewer lock when using its user_scn. Stops at maxgeom.
        """
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError("radius must be positive and finite")
        valid = self.valid & np.isfinite(self.heights)
        if height_range is not None:
            if (len(height_range) != 2 or not np.isfinite(height_range).all()
                    or height_range[0] >= height_range[1]):
                raise ValueError("height_range must contain two finite increasing heights")
            low, high = height_range
        elif valid.any():
            low, high = self.heights[valid].min(), self.heights[valid].max()
        else:
            return
        identity = np.eye(3).ravel()
        size = np.full(3, radius)
        for i, j in np.argwhere(valid):
            if scene.ngeom >= scene.maxgeom:
                return
            height = self.heights[i, j]
            fraction = np.clip((height - low) / (high - low), 0, 1) if high > low else 0.5
            rgb = colorsys.hsv_to_rgb((1 - fraction) * 2 / 3, 0.7, 1)
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom, mujoco.mjtGeom.mjGEOM_SPHERE, size,
                np.array([self.x[i, j], self.y[i, j], height]), identity,
                np.array([*rgb, 1.0], dtype=np.float32),
            )
            scene.ngeom += 1


class TerrainScanner:
    """Cast an N x M grid along world -Z, excluding a robot body subtree.

    ``robot_body`` must name the robot root, so every robot geom is excluded.
    The horizontal grid follows the heading of the body's X axis projected
    onto world XY. Roll and pitch do not tilt the grid or the vertical rays.
    If the body's X axis is vertical, heading is undefined and world X is used.
    ``spacing`` is in meters; rays start ``z_offset`` above the root body origin
    and extend at most ``max_distance`` meters. Robot geoms must be assigned to
    ``excluded_geom_groups`` (default 0 and 1, for visuals and collisions).
    Terrain must use a remaining group, such as 2. Any environment geoms in
    the excluded groups are also skipped.
    The scanner does not change the model's group assignments. Visible geoms
    in the other groups are scanned using MuJoCo's native ray intersection.

    Call scan on the simulation thread between steps. Reuse the scanner while
    the compiled model topology stays the same; no MJCF sensors are required.
    """

    def __init__(self, model: mujoco.MjModel, robot_body: str,
                 shape=(17, 11), spacing=(0.1, 0.1),
                 z_offset: float = 2.0, max_distance: float = 5.0,
                 excluded_geom_groups=(0, 1)):
        if not isinstance(robot_body, str) or not robot_body:
            raise ValueError("robot_body must be a non-empty robot root body name")
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
        if body_id <= 0:
            raise ValueError(f"Unknown robot body or world body: {robot_body!r}")
        if (not isinstance(shape, (tuple, list)) or len(shape) != 2
                or any(isinstance(n, (bool, np.bool_))
                       or not isinstance(n, (int, np.integer)) or n <= 0 for n in shape)):
            raise ValueError("shape must contain two positive integers [N, M]")
        if (not isinstance(spacing, (tuple, list)) or len(spacing) != 2):
            raise ValueError("spacing must contain two positive finite numbers")
        values = (*spacing, z_offset, max_distance)
        if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.number))
               or not np.isfinite(v) or v <= 0 for v in values):
            raise ValueError("spacing, z_offset and max_distance must be positive finite numbers")
        if (not isinstance(excluded_geom_groups, (tuple, list))
                or not excluded_geom_groups
                or any(isinstance(group, (bool, np.bool_))
                       or not isinstance(group, (int, np.integer))
                       or not 0 <= group < 6 for group in excluded_geom_groups)):
            raise ValueError("excluded_geom_groups must contain integers from 0 to 5")

        self.model = model
        self.body_id = body_id
        self.shape = tuple(shape)
        self.z_offset = float(z_offset)
        self.max_distance = float(max_distance)
        self._x = (np.arange(shape[0]) - (shape[0] - 1) / 2) * spacing[0]
        self._y = (np.arange(shape[1]) - (shape[1] - 1) / 2) * spacing[1]
        excluded = np.zeros(model.nbody, dtype=bool)
        excluded[body_id] = True
        # MuJoCo orders parents before children, including fixed and articulated links.
        for i in range(1, model.nbody):
            excluded[i] |= excluded[model.body_parentid[i]]
        robot_geoms = excluded[model.geom_bodyid]
        if not np.isin(model.geom_group[robot_geoms], excluded_geom_groups).all():
            raise ValueError(f"All robot geoms must use excluded groups {tuple(excluded_geom_groups)}")
        self._geomgroup = np.ones(6, dtype=np.uint8)
        self._geomgroup[list(excluded_geom_groups)] = 0

    def scan(self, data: mujoco.MjData) -> TerrainScan:
        # mj_step integrates qpos after computing derived poses. Refresh these
        # without advancing time or touching controls, velocities or forces.
        mujoco.mj_kinematics(self.model, data)
        center = data.xpos[self.body_id].copy()
        rotation = data.xmat[self.body_id].reshape(3, 3).copy()
        body_x = rotation[:2, 0]
        yaw = np.arctan2(body_x[1], body_x[0]) if np.linalg.norm(body_x) > 1e-12 else 0.0
        c, s = np.cos(yaw), np.sin(yaw)
        # Only yaw rotates the grid: every origin stays on the same world-Z plane.
        dx, dy = np.meshgrid(self._x, self._y, indexing='ij')
        x = center[0] + c * dx - s * dy
        y = center[1] + s * dx + c * dy
        start_z = float(center[2] + self.z_offset)
        heights = np.full(self.shape, np.nan)
        geom_ids = np.full(self.shape, -1, dtype=np.int32)
        direction = np.array([0.0, 0.0, -1.0])
        origin = np.empty(3)
        hit = np.empty(1, dtype=np.int32)
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                origin[:] = (x[i, j], y[i, j], start_z)
                distance = mujoco.mj_ray(
                    self.model, data, origin, direction, self._geomgroup, 1, -1, hit,
                )
                if 0 <= distance <= self.max_distance:
                    heights[i, j] = start_z - distance
                    geom_ids[i, j] = hit[0]
        return TerrainScan(float(data.time), center, x, y, start_z, heights, geom_ids, rotation)
