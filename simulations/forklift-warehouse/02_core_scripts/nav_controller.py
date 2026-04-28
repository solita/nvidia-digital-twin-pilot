"""
nav_controller.py — Waypoint navigation and drive command computation.

Owns:
  - Waypoint sequencing and arrival detection
  - Heading EMA smoothing
  - PD steer controller (proportional + derivative)
  - APF steer blending (PD heading attraction + LIDAR lateral repulsion)
  - Turn-speed limiter
  - Stuck detection and two-phase escape maneuver
  - Heading-loss detection and recovery spin

Does NOT touch sensor hardware, physics joints, or file I/O — changes here
never affect the LIDAR module and vice versa.

Dev owner: Navigation / path planning dev
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from pxr import Usd, UsdGeom

from fl_config import (
    DRIVE_VELOCITY, RAMP_FRAMES,
    STEER_KP, STEER_KD, STEER_DEADBAND, STEER_MAX, HEADING_SMOOTH,
    WAYPOINTS, ARRIVAL_RADIUS,
    LIDAR_HARD_STOP_DIST, LIDAR_STOP_DIST, LIDAR_SLOW_DIST, LIDAR_FWDSTOP_SPEED,
    STUCK_CHECK_FRAMES, STUCK_ESCAPE_FRAMES, STUCK_MIN_MOVE,
)
from lidar_processor import LidarResult


# ── Drive command dataclass ────────────────────────────────────────────────────

@dataclass
class DriveCommand:
    """Everything the main loop needs from one nav step.

    Joint setpoints:
        target_velocity  — set directly on drive joint (deg/s)
        steer_angle      — set directly on steer joint (deg, already sign-flipped
                           for the forks-forward joint axis convention)

    Telemetry (for state JSON + diag log):
        speed_frac       — 0.0–1.0 fraction of DRIVE_VELOCITY
        smooth_heading   — EMA-filtered yaw (deg)
        target_hdg       — bearing toward active waypoint (deg)
        heading_err      — signed heading error (deg)
        steer_cmd_pd     — PD-only steer before APF blend (for diag)
        wp_index         — active waypoint index
        lap              — completed lap count
        dist_to_wp       — distance to active waypoint (m)

    State flags:
        escape_just_started — True on the single frame escape begins (debounce reset signal)
        is_escaping         — True while escape maneuver is running
        is_recovering       — True while heading-loss recovery spin is running
    """
    target_velocity:     float
    steer_angle:         float
    speed_frac:          float
    smooth_heading:      float
    target_hdg:          float
    heading_err:         float
    steer_cmd_pd:        float
    wp_index:            int
    lap:                 int
    dist_to_wp:          float
    escape_just_started: bool = False
    is_escaping:         bool = False
    is_recovering:       bool = False


# ── NavController ──────────────────────────────────────────────────────────────

class NavController:
    """Stateful waypoint navigator.  Call step() once per physics frame."""

    def __init__(self) -> None:
        # Public — readable by main loop for logging on arrival
        self.wp_index: int = 0
        self.lap:      int = 0

        # Internal nav state
        self._frame:              int   = 0
        self._smooth_heading:     float | None = None  # initialised on first step
        self._prev_heading_err:   float = 0.0
        self._last_arrived_wp:    int   = -1            # wp just reached (for logging)

        # Stuck detection
        self._stuck_check_pos:    tuple | None = None
        self._stuck_frames:       int   = 0
        self._stuck_escape_frames: int  = 0
        self._stuck_wp_count:     int   = 0
        self._escape_just_started: bool = False

        # Heading-loss recovery
        self._lost_heading_count: int  = 0
        self._recovering_heading: bool = False

    # ── Public interface ───────────────────────────────────────────────────────

    @property
    def last_arrived_wp(self) -> int:
        """Index of the most recently reached waypoint (before incrementing)."""
        return self._last_arrived_wp

    def step(self, fx: float, fy: float, raw_yaw: float,
             lidar: LidarResult) -> DriveCommand | None:
        """Compute one drive command.

        Returns None when a waypoint was just reached — the main loop should
        skip setting joints and await the next physics update (matches the
        original 'continue' behaviour on arrival).
        """
        # Initialise heading EMA on first call
        if self._smooth_heading is None:
            self._smooth_heading = raw_yaw

        self._smooth_heading = (
            HEADING_SMOOTH * raw_yaw + (1.0 - HEADING_SMOOTH) * self._smooth_heading
        )

        wx, wy = WAYPOINTS[self.wp_index]
        dist   = math.hypot(wx - fx, wy - fy)

        # ── Arrival check ──────────────────────────────────────────────────────
        if dist < ARRIVAL_RADIUS:
            self._last_arrived_wp = self.wp_index
            self._stuck_wp_count  = 0
            self.wp_index        += 1
            if self.wp_index >= len(WAYPOINTS):
                self.wp_index = 0
                self.lap     += 1
            return None  # signal: arrived, skip this frame

        # ── Heading error ──────────────────────────────────────────────────────
        target_hdg  = math.degrees(math.atan2(wy - fy, wx - fx))
        # Forks point in body -X → their effective heading is smooth_heading + 180°
        heading_err = _angle_diff(self._smooth_heading + 180.0, target_hdg)

        # ── PD steer ───────────────────────────────────────────────────────────
        d_err     = heading_err - self._prev_heading_err
        steer_cmd = STEER_KP * heading_err + STEER_KD * d_err
        steer_cmd = max(-STEER_MAX, min(STEER_MAX, steer_cmd))
        if abs(heading_err) < STEER_DEADBAND:
            steer_cmd = 0.0
        self._prev_heading_err = heading_err

        # ── APF blend: PD attraction + LIDAR repulsion ─────────────────────────
        apf_steer = max(-STEER_MAX, min(STEER_MAX, steer_cmd + lidar.repulsion_steer))

        # ── Turn-speed limiter ─────────────────────────────────────────────────
        # 100 % speed when |err| ≤ 15°, ramps to 50 % at |err| ≥ 60°
        turn_scale = max(0.5, 1.0 - max(0.0, abs(heading_err) - 15.0) / 90.0)

        # ── Stuck detection → may override everything ──────────────────────────
        escape = self._update_stuck(fx, fy, heading_err, lidar.repulsion_steer)
        if escape is not None:
            vel, steer = escape
            just_started = self._escape_just_started
            self._escape_just_started = False
            self._frame += 1
            return DriveCommand(
                target_velocity=vel,
                steer_angle=steer,
                speed_frac=abs(vel / DRIVE_VELOCITY) if DRIVE_VELOCITY else 0.0,
                smooth_heading=self._smooth_heading,
                target_hdg=target_hdg,
                heading_err=heading_err,
                steer_cmd_pd=steer_cmd,
                wp_index=self.wp_index,
                lap=self.lap,
                dist_to_wp=dist,
                escape_just_started=just_started,
                is_escaping=True,
            )

        # ── Heading-loss recovery → may override everything ────────────────────
        recovery = self._update_heading_recovery(heading_err)
        if recovery is not None:
            vel, steer = recovery
            self._frame += 1
            return DriveCommand(
                target_velocity=vel,
                steer_angle=steer,
                speed_frac=abs(vel / DRIVE_VELOCITY) if DRIVE_VELOCITY else 0.0,
                smooth_heading=self._smooth_heading,
                target_hdg=target_hdg,
                heading_err=heading_err,
                steer_cmd_pd=steer_cmd,
                wp_index=self.wp_index,
                lap=self.lap,
                dist_to_wp=dist,
                is_recovering=True,
            )

        # ── Normal drive ───────────────────────────────────────────────────────
        if lidar.fwd_stop or lidar.fwd_slow:
            if lidar.forward_min < LIDAR_HARD_STOP_DIST:
                speed_frac = 0.0
            else:
                t = max(0.0, min(1.0,
                    (lidar.forward_min - LIDAR_STOP_DIST)
                    / max(LIDAR_SLOW_DIST - LIDAR_STOP_DIST, 0.1)
                ))
                speed_frac = LIDAR_FWDSTOP_SPEED + t * (1.0 - LIDAR_FWDSTOP_SPEED)
            target_vel  = DRIVE_VELOCITY * speed_frac * turn_scale
            final_steer = apf_steer
        else:
            ramp_scale  = min(1.0, self._frame / RAMP_FRAMES)
            speed_frac  = ramp_scale * turn_scale
            target_vel  = DRIVE_VELOCITY * speed_frac
            final_steer = apf_steer

        self._frame += 1
        return DriveCommand(
            target_velocity=target_vel,
            steer_angle=-final_steer,     # negated: forks-forward joint axis convention
            speed_frac=speed_frac,
            smooth_heading=self._smooth_heading,
            target_hdg=target_hdg,
            heading_err=heading_err,
            steer_cmd_pd=steer_cmd,
            wp_index=self.wp_index,
            lap=self.lap,
            dist_to_wp=dist,
        )

    # ── Stuck detection state machine ──────────────────────────────────────────

    def _update_stuck(self, fx: float, fy: float,
                      heading_err: float, repulsion_steer: float) -> tuple | None:
        """Return (velocity, steer_angle) while escape is running, else None.

        Also sets self._escape_just_started = True on the first escape frame
        so the main loop can signal LidarProcessor.reset_debounce().
        """
        # Don't track stuck position while we're already executing an escape
        if self._stuck_escape_frames <= 0:
            if self._stuck_check_pos is None:
                self._stuck_check_pos = (fx, fy)
                self._stuck_frames    = 0
            else:
                moved = math.hypot(fx - self._stuck_check_pos[0],
                                   fy - self._stuck_check_pos[1])
                if moved > STUCK_MIN_MOVE:
                    self._stuck_check_pos = (fx, fy)
                    self._stuck_frames    = 0
                else:
                    self._stuck_frames += 1
                    if self._stuck_frames == STUCK_CHECK_FRAMES:
                        self._stuck_wp_count += 1
                        if self._stuck_wp_count >= 5:
                            # Completely blocked — skip to next waypoint
                            self.wp_index         = (self.wp_index + 1) % len(WAYPOINTS)
                            self._stuck_wp_count  = 0
                            self._stuck_frames    = 0
                            self._stuck_check_pos = (fx, fy)
                        else:
                            # Start escape maneuver
                            self._stuck_escape_frames  = STUCK_ESCAPE_FRAMES
                            self._stuck_frames         = 0
                            self._stuck_check_pos      = (fx, fy)
                            self._escape_just_started  = True

        if self._stuck_escape_frames > 0:
            half = STUCK_ESCAPE_FRAMES // 2
            if self._stuck_escape_frames > half:
                # Phase 1: reverse straight back to break physical contact
                vel   =  abs(DRIVE_VELOCITY) * 0.40
                steer =  0.0
            else:
                # Phase 2: forward + open-side steer
                if abs(repulsion_steer) < 1.0:
                    escape_steer = STEER_MAX if heading_err > 0 else -STEER_MAX
                else:
                    escape_steer = STEER_MAX if repulsion_steer >= 0 else -STEER_MAX
                vel   = DRIVE_VELOCITY
                steer = -escape_steer   # negated for joint axis convention
            self._stuck_escape_frames -= 1
            return (vel, steer)

        return None

    # ── Heading-loss recovery state machine ────────────────────────────────────

    def _update_heading_recovery(self, heading_err: float) -> tuple | None:
        """Return (velocity, steer_angle) while recovery spin is running, else None."""
        if self._recovering_heading:
            if abs(heading_err) < 90.0:
                self._recovering_heading = False
                self._lost_heading_count = 0
                return None
            # Creep forward at low speed + full steer until back within 90° of target
            spin_steer = -STEER_MAX if heading_err < 0 else STEER_MAX
            return (DRIVE_VELOCITY * 0.20, -spin_steer)

        if abs(heading_err) > 150.0:
            self._lost_heading_count += 1
            if self._lost_heading_count >= 10:
                self._recovering_heading = True
        else:
            self._lost_heading_count = max(0, self._lost_heading_count - 2)

        return None


# ── USD helpers ────────────────────────────────────────────────────────────────

def get_world_transform(prim) -> tuple[float, float, float]:
    """Return (x, y, yaw_deg) of a USD prim in world space."""
    m   = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = m.ExtractTranslation()
    q   = m.ExtractRotationQuat()
    xi, yi, zi = q.GetImaginary()
    w   = q.GetReal()
    yaw = math.degrees(math.atan2(2.0 * (w * zi + xi * yi),
                                   1.0 - 2.0 * (yi * yi + zi * zi)))
    return float(pos[0]), float(pos[1]), yaw


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path difference (a - b), wrapped to [-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0
