"""
lidar_processor.py — LIDAR sensor lifecycle and per-frame depth processing.

Owns:
  - Sensor prim creation and interface acquisition
  - Per-frame depth data processing:
      * self-hit filtering (forklift bounding-box mask)
      * forward stop/slow with two-tier hit-count threshold and debounce
      * APF lateral repulsion (1/d per sector)
      * open-side directional bias
      * dashboard pie-chart slice flags
  - Internal debounce state

Does NOT touch joints, waypoints, or nav state — changes here never affect
the nav module and vice versa.

Dev owner: Sensor / perception dev
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import carb
import omni.kit.app
import omni.kit.commands
import omni.usd

from fl_config import (
    FORKLIFT_SAFE_HALF_FORKS, FORKLIFT_SAFE_HALF_LATERAL,
    LIDAR_AVOID_STEER, LIDAR_CONE_HALF, LIDAR_DEBOUNCE_FRAMES,
    LIDAR_DRAW_LINES, LIDAR_ENABLED, LIDAR_FAR_FLOOR,
    LIDAR_FWDSTOP_SPEED, LIDAR_FORWARD_RAY, LIDAR_HARD_STOP_DIST,
    LIDAR_MIN_HIT_COUNT_FAR, LIDAR_MIN_HIT_COUNT_NEAR,
    LIDAR_MIN_VALID, LIDAR_PRIM_PATH, LIDAR_REPULSE_ARC,
    LIDAR_REPULSE_GAIN, LIDAR_REPULSE_RANGE, LIDAR_SIDE_BACK_RANGE,
    LIDAR_SLOW_DIST, LIDAR_STOP_DIST, STEER_MAX,
)


# ── Result dataclass ───────────────────────────────────────────────────────────

@dataclass
class LidarResult:
    """All LIDAR-derived values for one physics frame."""
    forward_min:     float            # nearest validated forward-cone hit (m)
    fwd_stop:        bool             # debounced forward-obstacle stop
    fwd_slow:        bool             # obstacle within slow zone
    repulsion_steer: float            # APF lateral steer correction (deg)
    lidar_slices:    list = field(default_factory=lambda: [False] * 9)
    lidar_state:     str  = "CLEAR"   # "STOP" | "SLOW" | "CLEAR"
    debounce_count:  int  = 0         # exposed so escape logic can inspect it


# ── LidarProcessor ────────────────────────────────────────────────────────────

class LidarProcessor:
    """Creates and owns the LIDAR sensor; processes depth data each frame."""

    _ORIG_MAX_RANGE = 8.0  # factory default — used as the scaling baseline

    def __init__(self) -> None:
        self._interface          = None
        self._safe_zone: list    = _compute_safe_zone()
        self._debounce_count: int = 0
        self._clear_count: int   = 0

        # Mutable thresholds (initialised from fl_config, scaled by set_range)
        self.max_range       = self._ORIG_MAX_RANGE
        self.hard_stop_dist  = LIDAR_HARD_STOP_DIST
        self.stop_dist       = LIDAR_STOP_DIST
        self.slow_dist       = LIDAR_SLOW_DIST
        self.repulse_range   = LIDAR_REPULSE_RANGE
        self.side_back_range = LIDAR_SIDE_BACK_RANGE

    # ── One-time setup ─────────────────────────────────────────────────────────

    async def setup(self) -> None:
        """Create LIDAR prim and acquire sensor interface.

        Must be awaited once before the main loop starts.
        Safe to call even if LIDAR_ENABLED is False (no-op).
        """
        if not LIDAR_ENABLED:
            return

        app   = omni.kit.app.get_app()
        stage = omni.usd.get_context().get_stage()

        # Remove stale prim from a previous run (script re-runs without full reload)
        if stage.GetPrimAtPath(LIDAR_PRIM_PATH).IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[LIDAR_PRIM_PATH])
            await app.next_update_async()

        omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/lidar",                    # relative — lands at LIDAR_PRIM_PATH
            parent="/World/forklift_b/body",  # mounts on rigid body → moves with forklift
            min_range=0.5,
            max_range=8.0,
            draw_lines=LIDAR_DRAW_LINES,
            horizontal_fov=360.0,
            horizontal_resolution=1.0,        # 1 ray per degree → 360 rays total
            vertical_fov=0.0,                 # 2D single horizontal ring
            rotation_rate=0.0,                # sync to physics step
        )
        await app.next_update_async()

        try:
            from omni.isaac.range_sensor import _range_sensor
            self._interface = _range_sensor.acquire_lidar_sensor_interface()
            carb.log_info("[lidar] Sensor created and interface acquired")
        except Exception as exc:
            carb.log_warn(f"[lidar] Interface unavailable: {exc}")
            self._interface = None

    # ── Per-frame entry point ──────────────────────────────────────────────────

    def process(self) -> LidarResult:
        """Read and process one frame of LIDAR data.

        Returns a LidarResult with safe defaults if the sensor is disabled,
        the interface is unavailable, or a read error occurs.
        """
        empty = LidarResult(
            forward_min=9.9, fwd_stop=False, fwd_slow=False,
            repulsion_steer=0.0, debounce_count=self._debounce_count,
        )
        if not LIDAR_ENABLED or self._interface is None:
            return empty
        try:
            depths = self._interface.get_linear_depth_data(LIDAR_PRIM_PATH)
            if depths is None or depths.size < 360:
                return empty
            return self._process_depths(depths)
        except Exception as exc:
            carb.log_warn(f"[lidar] Read error: {exc}")
            return empty

    def reset_debounce(self) -> None:
        """Reset debounce counters — call at the start of a stuck-escape maneuver."""
        self._debounce_count = 0
        self._clear_count    = 0

    # ── Runtime range override ─────────────────────────────────────────────────

    def set_range(self, new_max: float) -> None:
        """Change LIDAR max range at runtime and scale all detection thresholds."""
        new_max = max(1.0, min(25.0, float(new_max)))
        ratio = new_max / self._ORIG_MAX_RANGE

        self.max_range       = new_max
        self.hard_stop_dist  = LIDAR_HARD_STOP_DIST  * ratio
        self.stop_dist       = LIDAR_STOP_DIST       * ratio
        self.slow_dist       = LIDAR_SLOW_DIST       * ratio
        self.repulse_range   = LIDAR_REPULSE_RANGE   * ratio
        self.side_back_range = LIDAR_SIDE_BACK_RANGE * ratio

        # Update the USD sensor prim so Isaac Sim respects the new range
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(LIDAR_PRIM_PATH)
            if prim.IsValid():
                attr = prim.GetAttribute("maxRange")
                if attr.IsValid():
                    attr.Set(new_max)
            carb.log_info(f"[lidar] Range → {new_max:.1f} m  "
                          f"(stop={self.stop_dist:.1f}, slow={self.slow_dist:.1f})")
        except Exception as exc:
            carb.log_warn(f"[lidar] Failed to update USD maxRange: {exc}")

    # ── Internal processing pipeline ───────────────────────────────────────────

    def _process_depths(self, depths) -> LidarResult:
        flat = [float(d) for d in depths.flat]
        n    = len(flat)

        # 1. Self-hit filter: discard depths inside the forklift bounding box
        for i in range(n):
            if flat[i] < self._safe_zone[i % 360]:
                flat[i] = float('inf')

        # 2. Side/back range cap: halve detection range outside forward cone
        for i in range(n):
            off = ((i - LIDAR_FORWARD_RAY) + 180) % 360 - 180
            if abs(off) > LIDAR_CONE_HALF and flat[i] > self.side_back_range:
                flat[i] = float('inf')

        # 3. Emergency close-range check (bypasses hit-count thresholds)
        #    2+ rays in tight ±6° cone reading < 2.5 m = real physical contact threat
        emerg_hits = [
            flat[i % n]
            for i in range(LIDAR_FORWARD_RAY - 6, LIDAR_FORWARD_RAY + 7)
            if math.isfinite(flat[i % n]) and 0.3 < flat[i % n] < self.hard_stop_dist
        ]
        forward_min = min(emerg_hits) if len(emerg_hits) >= 2 else 9.9

        # 4. Forward cone — two-tier hit-count threshold
        #    Near  0.8–4.5 m: narrow column fills ~5 rays  → threshold = 4
        #    Far   4.5–5.5 m: fork tines fill  ~8 rays  → threshold = 10; 1 m cube = ~11
        fwd_hits = [
            flat[i % n]
            for i in range(LIDAR_FORWARD_RAY - LIDAR_CONE_HALF,
                           LIDAR_FORWARD_RAY + LIDAR_CONE_HALF + 1)
            if math.isfinite(flat[i % n])
            and flat[i % n] > LIDAR_MIN_VALID
            and flat[i % n] < self.max_range
        ]
        near_stop = [d for d in fwd_hits if d < min(self.stop_dist, LIDAR_FAR_FLOOR)]
        far_stop  = [d for d in fwd_hits if LIDAR_FAR_FLOOR <= d < self.stop_dist]
        if (len(near_stop) >= LIDAR_MIN_HIT_COUNT_NEAR
                or len(far_stop) >= LIDAR_MIN_HIT_COUNT_FAR):
            if fwd_hits:
                forward_min = min(forward_min, min(fwd_hits))

        # 5. Hysteresis debounce: count up fast, count down only after 5 clear frames
        if forward_min < self.stop_dist:
            self._debounce_count = min(self._debounce_count + 1, LIDAR_DEBOUNCE_FRAMES)
            self._clear_count    = 0
        else:
            self._clear_count += 1
            if self._clear_count >= 5:
                self._debounce_count = max(self._debounce_count - 1, 0)

        fwd_stop = self._debounce_count >= LIDAR_DEBOUNCE_FRAMES
        fwd_slow = forward_min < self.slow_dist

        # 6. APF lateral repulsion
        repulsion = self._compute_repulsion(flat, n)
        repulsion = self._apply_open_side_bias(flat, n, fwd_stop, repulsion)

        # 7. Dashboard pie-chart slices
        lidar_slices = self._compute_slices(flat, n)

        lidar_state = "STOP" if fwd_stop else ("SLOW" if fwd_slow else "CLEAR")

        return LidarResult(
            forward_min=forward_min,
            fwd_stop=fwd_stop,
            fwd_slow=fwd_slow,
            repulsion_steer=repulsion,
            lidar_slices=lidar_slices,
            lidar_state=lidar_state,
            debounce_count=self._debounce_count,
        )

    def _compute_repulsion(self, flat: list, n: int) -> float:
        """APF 1/d lateral repulsion: find closest hit per side, compute net push."""
        left_min = right_min = 9.9
        for i in range(n):
            d = flat[i]
            if not math.isfinite(d) or d > self.repulse_range or d < 0.5:
                continue
            off = ((i - LIDAR_FORWARD_RAY) + 180) % 360 - 180
            if abs(off) < 10 or abs(off) > LIDAR_REPULSE_ARC:
                continue  # skip directly ahead and directly behind
            if abs(off) <= LIDAR_CONE_HALF and d < LIDAR_MIN_VALID:
                continue  # mast self-hit floor for near-forward rays
            if off > 0:
                right_min = min(right_min, d)
            else:
                left_min  = min(left_min,  d)
        r_left  = LIDAR_REPULSE_GAIN / left_min  if left_min  < self.repulse_range else 0.0
        r_right = LIDAR_REPULSE_GAIN / right_min if right_min < self.repulse_range else 0.0
        return max(-STEER_MAX, min(STEER_MAX, r_left - r_right))

    def _open_side_scan(self, flat: list, n: int) -> tuple[float, float]:
        """Return (right_min, left_min) in ±25-80° lateral arcs."""
        ro_r = min(
            (flat[i % n] for i in range(LIDAR_FORWARD_RAY + 25, LIDAR_FORWARD_RAY + 80)
             if math.isfinite(flat[i % n]) and flat[i % n] >= 0.5),
            default=9.9,
        )
        ro_l = min(
            (flat[i % n] for i in range(LIDAR_FORWARD_RAY - 80, LIDAR_FORWARD_RAY - 25)
             if math.isfinite(flat[i % n]) and flat[i % n] >= 0.5),
            default=9.9,
        )
        return ro_r, ro_l

    def _apply_open_side_bias(self, flat: list, n: int,
                               fwd_stop: bool, repulsion: float) -> float:
        """Blend a directional bias toward the more open lateral side."""
        if fwd_stop:
            ro_r, ro_l = self._open_side_scan(flat, n)
            bias = LIDAR_AVOID_STEER if ro_r >= ro_l else -LIDAR_AVOID_STEER
            return max(-STEER_MAX, min(STEER_MAX, repulsion + bias))
        if self._debounce_count >= 2:
            ro_r, ro_l = self._open_side_scan(flat, n)
            bias = (LIDAR_AVOID_STEER * 0.5) if ro_r >= ro_l else -(LIDAR_AVOID_STEER * 0.5)
            return max(-STEER_MAX, min(STEER_MAX, repulsion + bias))
        if self._debounce_count == 1 and abs(repulsion) < 5.0:
            ro_r, ro_l = self._open_side_scan(flat, n)
            return 8.0 if ro_r >= ro_l else -8.0
        return repulsion

    def _compute_slices(self, flat: list, n: int) -> list:
        """Return 9-element bool list for dashboard pie chart.

        Slices: [FL, FC, FR, RF, RB, BR, BL, LB, LF]
        """
        slices = [False] * 9
        for i in range(n):
            d = flat[i]
            if not math.isfinite(d) or d <= 0.80 or d >= 8.0:
                continue
            off = ((i - LIDAR_FORWARD_RAY) + 180) % 360 - 180
            if -20 <= off <= 20:
                if d < LIDAR_MIN_VALID:
                    continue
                if off < -6.67:
                    slices[0] = True
                elif off <= 6.67:
                    slices[1] = True
                else:
                    slices[2] = True
            elif 20 < off <= 73.33:
                slices[3] = True
            elif 73.33 < off <= 126.67:
                slices[4] = True
            elif off > 126.67:
                slices[5] = True
            elif off < -126.67:
                slices[6] = True
            elif -126.67 <= off < -73.33:
                slices[7] = True
            else:
                slices[8] = True
        return slices


# ── Module-level helper ────────────────────────────────────────────────────────

def _compute_safe_zone() -> list:
    """Per-ray (360) minimum distance to exit the forklift bounding box.

    Any LIDAR depth shorter than safe_zone[ray_index] is a self-hit on the
    forklift's own collider mesh and must be discarded.
    """
    safe = []
    for i in range(360):
        alpha = math.radians(((i - LIDAR_FORWARD_RAY) + 180) % 360 - 180)
        ca    = abs(math.cos(alpha))
        sa    = abs(math.sin(alpha))
        d_fwd = FORKLIFT_SAFE_HALF_FORKS   / ca if ca > 1e-6 else 999.0
        d_lat = FORKLIFT_SAFE_HALF_LATERAL / sa if sa > 1e-6 else 999.0
        safe.append(min(d_fwd, d_lat))
    return safe
