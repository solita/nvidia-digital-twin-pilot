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
    ESCAPE_BACKUP_SPEED, ESCAPE_BACKUP_EXTRA_DIST, ESCAPE_TURN_SPEED,
    ESCAPE_CIRCUMNAVIGATE_SPEED, ESCAPE_PHASE_TIMEOUT, ESCAPE_CIRCUMNAV_TIMEOUT,
    ESCAPE_FWD_CLEAR_DIST, WAREHOUSE_CENTER_X, WAREHOUSE_CENTER_Y,
    ESCAPE_BACKUP_INCREMENT, ESCAPE_TURN_INCREMENT, ESCAPE_MAX_ATTEMPTS,
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
        self.disable_evasion: bool = False  # hard-follow path, no obstacle avoidance

        # Internal nav state
        self._frame:              int   = 0
        self._smooth_heading:     float | None = None  # initialised on first step
        self._prev_heading_err:   float = 0.0
        self._last_arrived_wp:    int   = -1            # wp just reached (for logging)

        # Stuck detection
        self._stuck_check_pos:    tuple | None = None
        self._stuck_frames:       int   = 0
        self._stuck_wp_count:     int   = 0
        self._escape_just_started: bool = False

        # 4-phase escape state
        self._escape_phase:       int   = 0   # 0=inactive, 1=backup, 2=turn, 3=circumnavigate
        self._escape_phase_frames: int  = 0   # frames spent in current phase
        self._escape_clear_pos:   tuple | None = None  # position when tines cleared
        self._escape_turn_dir:    float = 0.0  # +1 or -1: direction to turn toward centre
        self._escape_side_sign:   float = 0.0  # +1=object on right, -1=object on left

        # Incremental escalation: each re-stuck increases backup and turn aggression
        self._escape_attempt:     int   = 0   # how many times we've retried on same obstacle

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
            self._escape_attempt  = 0  # successfully moved on — reset escalation
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
        if self.disable_evasion:
            apf_steer = steer_cmd  # no lateral repulsion — hard follow path
        else:
            apf_steer = max(-STEER_MAX, min(STEER_MAX, steer_cmd + lidar.repulsion_steer))

        # ── Turn-speed limiter ─────────────────────────────────────────────────
        # 100 % speed when |err| ≤ 15°, ramps to 50 % at |err| ≥ 60°
        turn_scale = max(0.5, 1.0 - max(0.0, abs(heading_err) - 15.0) / 90.0)

        # ── Stuck detection → may override everything ──────────────────────────
        if not self.disable_evasion:
            escape = self._update_stuck(fx, fy, heading_err, lidar)
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
        if not self.disable_evasion:
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
        if not self.disable_evasion and (lidar.fwd_stop or lidar.fwd_slow):
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
                      heading_err: float, lidar: LidarResult) -> tuple | None:
        """Return (velocity, steer_angle) while escape is running, else None.

        4-phase lidar-guided escape maneuver:
          Phase 1 (BACKUP):  Reverse until forward cone clears, then 1 extra metre.
          Phase 2 (TURN):    Rotate toward warehouse centre until front cone is clear.
          Phase 3 (CIRCUMNAV): Drive forward keeping object on one side via lidar.
          Phase 4 (done):    Return None → normal path-following resumes.

        Also sets self._escape_just_started = True on the first escape frame
        so the main loop can signal LidarProcessor.reset_debounce().
        """
        # ── If escape is active, run the phase state machine ───────────────────
        if self._escape_phase > 0:
            return self._run_escape_phase(fx, fy, lidar)

        # ── Otherwise, detect whether we are stuck ─────────────────────────────
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
                if self._stuck_frames >= STUCK_CHECK_FRAMES:
                    self._escape_attempt += 1
                    self._stuck_wp_count += 1
                    if (self._stuck_wp_count >= 5
                            or self._escape_attempt > ESCAPE_MAX_ATTEMPTS):
                        # Completely blocked — skip waypoint, reset escalation
                        self.wp_index         = (self.wp_index + 1) % len(WAYPOINTS)
                        self._stuck_wp_count  = 0
                        self._escape_attempt  = 0
                        self._stuck_frames    = 0
                        self._stuck_check_pos = (fx, fy)
                    else:
                        # Start 4-phase escape with escalated parameters
                        self._begin_escape(fx, fy, lidar)
                        return self._run_escape_phase(fx, fy, lidar)

        return None

    def _begin_escape(self, fx: float, fy: float, lidar: LidarResult) -> None:
        """Initialise the 4-phase escape maneuver."""
        self._escape_phase        = 1  # start with backup
        self._escape_phase_frames = 0
        self._escape_clear_pos    = None
        self._stuck_frames        = 0
        self._stuck_check_pos     = (fx, fy)
        self._escape_just_started = True

        # Determine turn direction: toward warehouse centre
        bearing_to_centre = math.degrees(
            math.atan2(WAREHOUSE_CENTER_Y - fy, WAREHOUSE_CENTER_X - fx)
        )
        # Forklift forward heading (forks point body -X → effective heading + 180)
        fwd_heading = (self._smooth_heading or 0.0) + 180.0
        angle_to_centre = _angle_diff(bearing_to_centre, fwd_heading)
        self._escape_turn_dir = 1.0 if angle_to_centre >= 0 else -1.0

        # Determine which side the object is on (for circumnavigation)
        # If turning right (+steer), object ends up on the left side and vice versa
        self._escape_side_sign = -self._escape_turn_dir  # +1=obj right, -1=obj left

    def _run_escape_phase(self, fx: float, fy: float,
                          lidar: LidarResult) -> tuple | None:
        """Execute the current escape phase and return (velocity, steer_angle).

        Returns None if the escape maneuver is complete (transition to normal nav).
        """
        self._escape_phase_frames += 1
        just_started = self._escape_just_started
        self._escape_just_started = False

        if self._escape_phase == 1:
            result = self._escape_phase_backup(fx, fy, lidar)
        elif self._escape_phase == 2:
            result = self._escape_phase_turn(fx, fy, lidar)
        elif self._escape_phase == 3:
            result = self._escape_phase_circumnavigate(fx, fy, lidar)
        else:
            # Escape complete
            self._escape_phase = 0
            return None

        if result is None:
            # Phase signalled completion — escape done
            self._escape_phase = 0
            return None

        # Re-set the just_started flag so the first call propagates it
        if just_started:
            self._escape_just_started = True

        return result

    def _advance_phase(self) -> None:
        """Move to next escape phase, reset per-phase frame counter."""
        self._escape_phase       += 1
        self._escape_phase_frames = 0

    # ── Phase 1: BACKUP until tines clear + escalated extra distance ──────────

    def _escape_phase_backup(self, fx: float, fy: float,
                             lidar: LidarResult) -> tuple | None:
        """Reverse until forward cone clears, then back up (base + attempt*1m) more."""
        vel   = abs(DRIVE_VELOCITY) * ESCAPE_BACKUP_SPEED  # positive = reverse
        steer = 0.0  # straight back

        # Escalated backup distance: base + 1 m per attempt
        required_extra = (ESCAPE_BACKUP_EXTRA_DIST
                          + ESCAPE_BACKUP_INCREMENT * (self._escape_attempt - 1))

        # Timeout scales with distance (more backup = more time allowed)
        timeout = ESCAPE_PHASE_TIMEOUT + 60 * (self._escape_attempt - 1)
        if self._escape_phase_frames >= timeout:
            self._advance_phase()
            return (vel, steer)

        # Check if tines have cleared (forward cone no longer detecting close object)
        tines_clear = lidar.forward_min >= LIDAR_STOP_DIST or not lidar.fwd_stop

        if tines_clear and self._escape_clear_pos is None:
            # Mark the position where we first cleared
            self._escape_clear_pos = (fx, fy)

        if self._escape_clear_pos is not None:
            # Continue backing until escalated extra distance from the clearing point
            extra_dist = math.hypot(fx - self._escape_clear_pos[0],
                                    fy - self._escape_clear_pos[1])
            if extra_dist >= required_extra:
                self._advance_phase()
                return (vel, steer)

        return (vel, steer)

    # ── Phase 2: TURN toward warehouse centre until cone is clear ─────────────

    def _escape_phase_turn(self, fx: float, fy: float,
                           lidar: LidarResult) -> tuple | None:
        """Rotate toward warehouse centre with escalating aggressiveness.

        Each retry increases the turn steer multiplier so the forklift swings
        wider around larger objects.
        """
        # Escalated turn intensity: base full steer + increment per attempt
        turn_mult = min(1.0, 1.0 + ESCAPE_TURN_INCREMENT * (self._escape_attempt - 1))
        # Increase creep speed slightly on retries so the arc radius grows
        creep_speed = ESCAPE_TURN_SPEED * (1.0 + 0.15 * (self._escape_attempt - 1))
        creep_speed = min(creep_speed, 0.50)  # cap at 50 %

        vel   = DRIVE_VELOCITY * creep_speed
        steer = -(self._escape_turn_dir * STEER_MAX * turn_mult)  # negated for joint axis
        # Clamp to joint limits
        steer = max(-STEER_MAX, min(STEER_MAX, steer))

        # Timeout scales with attempt (more time for bigger turns)
        timeout = ESCAPE_PHASE_TIMEOUT + 80 * (self._escape_attempt - 1)
        if self._escape_phase_frames >= timeout:
            self._advance_phase()
            return (vel, steer)

        # Transition: forward cone is completely clear
        if (not lidar.fwd_stop
                and lidar.forward_min >= ESCAPE_FWD_CLEAR_DIST
                and self._escape_phase_frames > 10):  # min frames to avoid false clear
            self._advance_phase()
            return (vel, steer)

        return (vel, steer)

    # ── Phase 3: CIRCUMNAVIGATE — drive past object keeping it on one side ────

    def _escape_phase_circumnavigate(self, fx: float, fy: float,
                                     lidar: LidarResult) -> tuple | None:
        """Drive forward keeping the obstacle between lidar side edge and body.

        Object should be detected on one lateral side but NOT in the forward cone.
        When the side sensors no longer detect the object, the obstacle has been passed.
        """
        vel = DRIVE_VELOCITY * ESCAPE_CIRCUMNAVIGATE_SPEED

        # Timeout fallback
        if self._escape_phase_frames >= ESCAPE_CIRCUMNAV_TIMEOUT:
            self._escape_phase = 0  # end escape entirely
            return None

        # Determine if object is still on the expected side using lidar slices
        # lidar_slices: [FL, FC, FR, RF, RB, BR, BL, LB, LF]
        #   RF=index 3 (right-front), LF=index 8 (left-front)
        #   FR=index 2 (front-right), FL=index 0 (front-left)
        if self._escape_side_sign > 0:
            # Object should be on the right side
            object_on_side = lidar.lidar_slices[3] or lidar.lidar_slices[2]
        else:
            # Object should be on the left side
            object_on_side = lidar.lidar_slices[8] or lidar.lidar_slices[0]

        # Gentle steer away from the object side to maintain clearance
        # (small steer away keeps a safe corridor)
        if object_on_side:
            steer = -(self._escape_turn_dir * STEER_MAX * 0.3)
        else:
            # Object no longer on side — drive straight briefly then exit
            steer = 0.0

        # If forward cone detects something, add corrective steer away
        if lidar.fwd_stop or lidar.forward_min < LIDAR_STOP_DIST:
            steer = -(self._escape_turn_dir * STEER_MAX * 0.6)
            vel   = DRIVE_VELOCITY * ESCAPE_TURN_SPEED

        # Transition: object no longer on side AND forward is clear → passed it
        if (not object_on_side
                and not lidar.fwd_stop
                and lidar.forward_min >= ESCAPE_FWD_CLEAR_DIST
                and self._escape_phase_frames > 30):
            # Escape complete — successfully passed the object, reset escalation
            self._escape_phase    = 0
            self._escape_attempt  = 0
            self._stuck_check_pos = (fx, fy)
            return None

        return (vel, steer)

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
