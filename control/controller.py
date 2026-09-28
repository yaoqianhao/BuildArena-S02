"""S02 Rocket Orbit-Return controller: full 3-lap orbit, deorbit and recovery.

Rebuilt from ``control/examples/rocket_orbit_return/controller.py`` (same
LONE ORB environment model, same Frame / AttitudeController / ThrottlePwm /
mission-sequencer scaffolding) and extended with the S02 gate requirements.

Machine (rebuilt from ``machine.json``, 553 history operations):

    Starting Block                     ID 1    (root / avionics)
    Fuel Barrel Big x4                 ID 2-5  (one central ~400-unit stack)
    Booster x4                         ID 6-9  (fixed fthrust, ThrustKey PWM)
    Grid Fin x4                        ID 10-13(aero stability)
    Reaction Steering Block x2         ID 46-47(two torque axes, LeftKey/RightKey)
    Landing legs (Ballast+Suspension+  ID 16-27
      Grip Pad x4)
    Braces / fuel lines                many

Single stage, no engine gimbal, no decoupler: the only actuators are the
four boosters (Bresenham PWM on ThrustKey) and the two reaction wheels.
``discover_hardware`` also probes for Decoupler blocks; if any are found the
controller automatically arms the separation branch (``separate`` stage),
otherwise the phase chain is the single-stage one below.

S02 phase chain (one closed-loop episode):

    pad -> guided_ascent -> orbit_coast -> kill_burn -> final_descent ->
    touchdown -> settled_hold (10 s) -> done

Optional separation branch (armed only when a Decoupler exists):

    ... orbit_coast -> separate -> payload_coast -> done

S02 scoring gates (all checked inside the controller / mission summary):

    * orbit gate : accumulated sweep >= 1080 deg (6pi / 3 laps), with
                   periapsis radius >= 1100 (planet-center distance)
    * recovery   : machine integrity >= 0.90 and a settled landing held
                   for >= 10 s (no engine, total speed <= 0.5)

Environment: LONE ORB Newtonian point-gravity sandbox.  Scene GM / center /
surface radius stay explicit constants because gravity fields are not part
of the configurable v4 telemetry contract.

Calibration notes / assumptions (verify on the first live flight):

    * thrust_accel = 36 u/s^2 (4 boosters at game-default fthrust=1.0) was
      re-measured on this exact rebuilt stack by the original demo; if the
      rebuild changes mass, re-run with HEAVY_LAUNCHER_POLICY=calibrate.
    * Wheel torque axes are bound dynamically from the machine-frame offsets
      of the first telemetry sample (LeftKey torque = -offset unit vector,
      RightKey = the opposite).  The machine.json does not encode the wheel
      mounting flip, so if a live calibration run shows attitude divergence,
      overwrite WHEEL_CALIBRATION_OVERRIDE below with the measured axes
      (see analyze_calibration output in the run directory).

Usage:
    uv run python -m control.run_example s02_rocket_orbit_return --prepare-only
    uv run python -m control.run_example s02_rocket_orbit_return
    # calibration policy:
    $env:HEAVY_LAUNCHER_POLICY = "calibrate"
    uv run python -m control.run_example s02_rocket_orbit_return
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

from controller_sdk import ControllerClient
from controller_sdk.protocol import ENV_CHANNEL_CATALOG, ENV_MACHINE_BSG, ENV_RUN_DIR

ROOT_GUID = ""
ENGINE_GUIDS: tuple[str, ...] = ()
WHEEL_GUIDS: tuple[str, ...] = ()
DECOUPLER_GUID = ""
WHEEL_A_GUID = ""
WHEEL_B_GUID = ""
# Exact MSlider.Key published live by ThrusterBlockBehaviour.
BOOSTER_POWER_SLIDER = "fthrust"
# Live key on Reaction Steering Block (id 101).  Authored docs used to say
# rotation-speed; that string is what Grid Fins publish, not the wheels.
WHEEL_SPEED_SLIDER = "speed"
# Live key on Decoupler (id 4): pressing it fires the separation action.
DECOUPLER_SEPARATION_KEY = "ExplodeKey"
# Optional hard-coded wheel axes (unit vectors, machine frame) for LeftKey.
# Leave empty to bind dynamically from the first telemetry sample.
WHEEL_CALIBRATION_OVERRIDE: dict[str, tuple[float, float, float]] = {}
# Machine-frame torque axis of holding LeftKey on each wheel (unit vector);
# RightKey is the exact opposite.  Rebound dynamically by bind_wheel_axes.
WHEEL_CALIBRATION: dict[str, dict[str, tuple[float, float, float]]] = {}
# Scene constants are explicit inputs, not telemetry fallbacks.  Recalibrate
# them before using this controller in another sandbox.
LONE_ORB_CENTER = (0.0, -600.0, 0.0)
LONE_ORB_SURFACE_RADIUS = 600.0
LONE_ORB_GM = 8_456_800.0
LONE_ORB_MAX_GRAVITY = 22.0


# ---------------------------------------------------------------------------
# Small vector helpers (tuples in, tuples out; no numpy in the 10 Hz loop)
# ---------------------------------------------------------------------------

def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_norm(a):
    return math.sqrt(v_dot(a, a))


def v_unit(a):
    n = v_norm(a)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    return v_scale(a, 1.0 / n)


def q_rotate(q, v):
    """Rotate vector v by quaternion q=(x,y,z,w) (Unity component order)."""
    x, y, z, w = q
    ux, uy, uz = (x, y, z)
    s = w
    uv = v_cross((ux, uy, uz), v)
    uuv = v_cross((ux, uy, uz), uv)
    return v_add(v, v_add(v_scale(uv, 2.0 * s), v_scale(uuv, 2.0)))


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


# ---------------------------------------------------------------------------
# Telemetry frame
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    """One decoded BAT3 root-block sample plus derived orbit kinematics."""

    t: float
    pos: tuple
    vel: tuple
    ang_vel: tuple
    rot: tuple
    gravity_center: tuple
    gravity_acc: tuple
    gravity_available: bool
    surface_radius: float
    fuel_fraction: float
    block_count: int
    machine_integrity: float
    # derived
    rel: tuple = (0.0, 0.0, 0.0)
    radius: float = 0.0
    up_r: tuple = (0.0, 1.0, 0.0)
    v_rad: float = 0.0
    v_tan_vec: tuple = (0.0, 0.0, 0.0)
    v_tan: float = 0.0
    speed: float = 0.0
    g_mag: float = 0.0
    up_axis: tuple = (0.0, 1.0, 0.0)  # rocket long axis (world)

    @staticmethod
    def fuel_fraction_from_sample(sample) -> float:
        """Controller-derived remaining/capacity over every fuel-valid block.

        Robust to separation: if the root keeps no fuel-valid block (payload
        case) capacity == 0 and we return 0.0 instead of raising.
        """
        names = set(sample.records.dtype.names or ())
        if "fuel" not in names or "fuel_valid" not in names:
            raise ValueError("BAT4 sample is missing per-block fuel; use profile full.")
        remaining = 0.0
        capacity = 0.0
        for row in sample.records:
            if int(row["fuel_valid"]) != 1:
                continue
            tank = row["fuel"]
            remaining += float(tank[0])
            capacity += float(tank[1])
        if capacity <= 0.0:
            return 0.0
        return remaining / capacity

    @staticmethod
    def from_telemetry(sample, record) -> "Frame":
        names = set(record.dtype.names or ())
        required = {"position", "rotation", "velocity", "angular_velocity"}
        if not required.issubset(names):
            raise ValueError(
                f"BAT3 root record is missing fields {sorted(required - names)}."
            )
        pos = tuple(float(value) for value in record["position"])
        vel = tuple(float(value) for value in record["velocity"])
        rot = tuple(float(value) for value in record["rotation"])
        ang_vel = tuple(float(value) for value in record["angular_velocity"])
        rel = v_sub(pos, LONE_ORB_CENTER)
        radius = v_norm(rel)
        if radius <= 1e-6:
            raise ValueError("Root block is at the configured gravity center.")
        up_r = v_scale(rel, 1.0 / radius)
        g_mag = min(LONE_ORB_MAX_GRAVITY, LONE_ORB_GM / (radius * radius))
        gravity_acc = v_scale(up_r, -g_mag)
        machine = sample.machine
        if machine.machine_integrity is None or machine.alive_block_count is None:
            raise ValueError(
                "Full BAT4 telemetry did not provide machine integrity fields."
            )
        frame = Frame(
            t=float(sample.simulation_time),
            pos=pos,
            vel=vel,
            ang_vel=ang_vel,
            rot=rot,
            gravity_center=LONE_ORB_CENTER,
            gravity_acc=gravity_acc,
            gravity_available=True,
            surface_radius=LONE_ORB_SURFACE_RADIUS,
            fuel_fraction=Frame.fuel_fraction_from_sample(sample),
            block_count=int(machine.alive_block_count),
            machine_integrity=float(machine.machine_integrity),
        )
        frame.rel = rel
        frame.radius = radius
        frame.up_r = up_r
        frame.v_rad = v_dot(frame.vel, frame.up_r)
        frame.v_tan_vec = v_sub(frame.vel, v_scale(frame.up_r, frame.v_rad))
        frame.v_tan = v_norm(frame.v_tan_vec)
        frame.speed = v_norm(frame.vel)
        frame.g_mag = g_mag
        frame.up_axis = v_unit(q_rotate(frame.rot, (0.0, 1.0, 0.0)))
        return frame


# ---------------------------------------------------------------------------
# Attitude controller (active two-axis reaction-wheel steering)
# ---------------------------------------------------------------------------

@dataclass
class AttitudeController:
    """Point the rocket's long axis at a desired world direction using the
    two reaction wheels (LeftKey/RightKey = +/- torque about a machine-frame
    axis; auto-brake damps when released).

    Control law: omega_des = kp * (u x d) with lookahead-predicted u; each
    wheel receives the projection of (omega_des - omega) onto its own world
    torque axis and fires bang-bang outside a deadband.
    """

    kp: float
    kd: float
    deadband: float
    lookahead: float

    def commands(self, *, frame: Frame, desired_dir: tuple) -> tuple[list[tuple[str, str]], dict]:
        u = frame.up_axis
        omega = frame.ang_vel
        # first-order latency compensation: u advances along omega x u
        u_pred = v_unit(v_add(u, v_scale(v_cross(omega, u), self.lookahead)))
        err = v_cross(u_pred, desired_dir)
        omega_des = v_scale(err, self.kp)
        torque_req = v_sub(omega_des, v_scale(omega, self.kd))
        controls: list[tuple[str, str]] = []
        debug = {"att_err": v_norm(err), "align": v_dot(u_pred, desired_dir)}
        for guid, cal in WHEEL_CALIBRATION.items():
            axis_world = v_unit(q_rotate(frame.rot, cal["axis"]))
            s = v_dot(torque_req, axis_world)
            if s > self.deadband:
                controls.append((guid, "LeftKey"))
                debug[f"wheel_{guid[:4]}"] = 1.0
            elif s < -self.deadband:
                controls.append((guid, "RightKey"))
                debug[f"wheel_{guid[:4]}"] = -1.0
            else:
                debug[f"wheel_{guid[:4]}"] = 0.0
        return controls, debug


class ThrottlePwm:
    """Bresenham duty-cycle accumulator: fires the engines on a fraction of
    10 Hz ticks so the fixed-thrust boosters approximate a throttle."""

    def __init__(self):
        self.accumulator = 0.0

    def fire(self, *, duty: float) -> bool:
        duty = max(0.0, min(1.0, duty))
        self.accumulator += duty
        if self.accumulator >= 1.0 - 1e-9:
            self.accumulator -= 1.0
            return True
        return False


def discover_booster_power_sliders(*, env: ControllerClient) -> list:
    """Return the live ``fthrust`` slider on every discovered Booster."""
    wanted = {guid.lower() for guid in ENGINE_GUIDS}
    found: list = []
    published: list[str] = []
    for slider in env.sliders:
        if slider.block_guid.lower() not in wanted:
            continue
        published.append(
            f"{slider.block_guid} {slider.name} "
            f"[{slider.minimum},{slider.maximum}] default={slider.default}"
        )
        if slider.name == BOOSTER_POWER_SLIDER:
            found.append(slider)
    if len(found) != len(ENGINE_GUIDS):
        raise RuntimeError(
            f"Each Booster must publish slider {BOOSTER_POWER_SLIDER!r}; "
            f"found {len(found)} for {len(ENGINE_GUIDS)} engines. "
            f"Published booster sliders: {published or '(none)'}."
        )
    return found


def discover_wheel_speed_sliders(*, env: ControllerClient) -> list:
    """Return the live ``speed`` slider on every discovered reaction wheel."""
    wanted = {guid.lower() for guid in WHEEL_GUIDS}
    found: list = []
    published: list[str] = []
    for slider in env.sliders:
        if slider.block_guid.lower() not in wanted:
            continue
        published.append(
            f"{slider.block_guid} {slider.name} "
            f"[{slider.minimum},{slider.maximum}] default={slider.default}"
        )
        if slider.name == WHEEL_SPEED_SLIDER:
            found.append(slider)
    if len(found) != len(WHEEL_GUIDS):
        raise RuntimeError(
            f"Each Reaction Steering Block must publish slider {WHEEL_SPEED_SLIDER!r}; "
            f"found {len(found)} for {len(WHEEL_GUIDS)} wheels. "
            f"Published wheel sliders: {published or '(none)'}."
        )
    return found


def pin_operating_sliders(
    *,
    env: ControllerClient,
    assignments: list[tuple[object, float]],
) -> None:
    """Pin sliders in one BAA4 snapshot.  Later key sends keep this snapshot."""
    if not assignments:
        raise RuntimeError("pin_operating_sliders requires at least one slider.")
    records: list[tuple[str, str, float]] = []
    for slider, value in assignments:
        value = float(value)
        if value < slider.minimum or value > slider.maximum:
            raise RuntimeError(
                f"Pin {slider.name}={value} on {slider.block_guid} is outside "
                f"[{slider.minimum},{slider.maximum}]."
            )
        records.append((slider.block_guid, slider.name, value))
        print(
            f"Pinned {slider.name}={value} on {slider.block_guid} "
            f"(range [{slider.minimum},{slider.maximum}] default={slider.default})",
            flush=True,
        )
    env.send_sliders(sliders=records)


# ---------------------------------------------------------------------------
# Mission sequencer
# ---------------------------------------------------------------------------

@dataclass
class MissionConfig:
    # S02 gate-machine scenario constants (LONE ORB).
    # Insertion target radius 1150 (> S02 gate 1100) leaves margin for drag
    # and for the 3-lap maintenance drift; 550 = 1150 - 600.
    target_altitude: float = 550.0
    # -- guided ascent (velocity-vector steering; reused from rocket demo) --
    # vr_target = clamp(gain*(r_t - r), vr_min, vr_max); vr_min is NEGATIVE so
    # the guidance commands descent after overshooting the target radius.
    ascent_vr_gain: float = 0.25
    ascent_vr_min: float = -8.0
    ascent_vr_max: float = 12.0
    ascent_vr_kp: float = 2.5
    ascent_vt_kp: float = 4.0
    ascent_drag_ff: float = 10.0
    ascent_vr_authority: float = 25.0     # |a_rad| cap of the vr feedback term
    ascent_slew_deg_s: float = 10.0       # max rotation rate of the guidance dir
    # Above this tangential speed the aero weathervane torque dominates and
    # the attitude is effectively locked to the velocity: switch to zero-AoA
    # throttle-only energy management.
    throttle_mode_vtan: float = 60.0
    throttle_ff_duty: float = 0.30        # ~drag/thrust at ~100 m/s, AoA ~0
    throttle_ke: float = 0.6              # duty per 1000 u^2/s^2 energy deficit
    throttle_kvr: float = 0.03            # duty cut per m/s of climb rate
    orbit_peri_frac: float = 0.92         # peri >= surface + frac*target_alt
    orbit_apo_frac: float = 0.80          # apo >= surface + frac*target_alt
    # -- S02 orbit gate -----------------------------------------------------
    # A full 3-lap sweep is required; peri/radius must clear 1100 (the S02
    # planet-center-distance gate) during the whole coast.
    orbit_sweep_deg: float = 1080.0       # 6pi: three full laps
    s02_peri_threshold: float = 1100.0    # hard periapsis radius gate
    s02_radius_threshold: float = 1100.0  # hard instantaneous radius gate
    # Insertion margin: the orbit-coast phase only *maintains* the ellipse
    # it is handed, so the insertion gate must clear the S02 periapsis gate
    # with headroom instead of touching it exactly.  The failed flight
    # inserted at peri=991.6 via powered_ok (which did NOT check periapsis)
    # and min_coast_peri kept that low value for all 3 laps.
    s02_peri_margin: float = 30.0         # insertion requires peri >= 1130
    # Coast maintenance target periapsis radius.  Lower than the insertion
    # radius to save fuel for the deorbit / landing.  It is interpreted as
    # a *periapsis* target, not a circular-orbit radius: the energy target
    # is -GM/(r + peri_target), so when the vehicle is near apoapsis the
    # resulting orbit has periapsis == peri_target (>= 1100) instead of
    # collapsing back below the gate.
    raise_target_radius: float = 1150.0
    coast_radius: float = 1120.0
    coast_early_exit_fuel: float = 0.12   # bail to deorbit if this low after 1 lap
    # -- deorbit (kill_burn) ------------------------------------------------
    # Retrograde braking only.  The previous hover-brake law (radial hold +
    # tilt) pumped energy into the orbit at orbital speed and climbed into
    # escaped_altitude_bound (v_rad went +102 while braking).  Attitude gate
    # keeps the throttle shut until the body is nearly aligned with -v_tan,
    # otherwise the early burn accelerates the orbit instead of braking it.
    kill_burn_exit_vtan: float = 6.0      # hand over to final_descent below this
    kill_burn_kv: float = 0.9             # retro accel per m/s of tangential excess
    kill_burn_align_gate: float = 0.85    # cos(align) to open full throttle
    # -- S02 recovery gate --------------------------------------------------
    required_integrity: float = 0.90      # machine integrity at touchdown
    settled_hold_sec: float = 10.0        # settled (speed<=0.5) hold after landing
    # -- separation (auto-armed when a Decoupler is discovered) -------------
    separation_block_drop: int = 5        # alive blocks must drop by this much
    separation_timeout: float = 8.0
    # -- attitude loop (two orthogonal wheels on this machine) --------------
    att_kp: float = 0.55
    att_kd: float = 2.2
    att_deadband: float = 0.04
    att_lookahead: float = 0.35
    use_active_attitude: bool = True
    # -- propulsion model ---------------------------------------------------
    # Calibrated on the original rebuilt stack (4 boosters at fthrust=1.0,
    # ~39.15 mass blocks): specific thrust ~36 u/s^2, hover duty ~g/36=0.61.
    # If the S02 rebuild changes mass, re-run with HEAVY_LAUNCHER_POLICY=
    # calibrate and update this value.
    thrust_accel: float = 36.0
    full_thrust_accel: float = 36.0
    # -- landing ------------------------------------------------------------
    tilt_gain: float = 0.06
    tilt_max_deg: float = 10.0
    final_descent_gain: float = 0.22
    final_min_sink: float = 1.5
    final_duty_kp: float = 0.10
    final_touchdown_speed: float = 2.0
    # -- limits -------------------------------------------------------------
    max_duration: float = 1800.0


@dataclass
class MissionController:
    """Discrete stage machine that turns telemetry into duty / attitude / keys.

    S02 phase chain for this single-stage VTVL rocket:

        pad -> guided_ascent -> orbit_coast -> kill_burn -> final_descent ->
        touchdown -> settled_hold -> done

    A ``separate -> payload_coast`` branch is inserted after orbit_coast only
    when a Decoupler block was discovered (``config.separation_enabled``).
    """

    config: MissionConfig
    stage: str = "pad"
    stage_started: float = 0.0
    done: bool = False
    abort_reason: str | None = None
    events: list[dict] = field(default_factory=list)

    surface_radius: float = 0.0
    pad_radius: float = 0.0
    ground_clearance: float = 0.0
    initial_blocks: int = 0
    max_altitude: float = 0.0
    gm: float = 0.0
    east: tuple | None = None
    guidance_dir: tuple = (0.0, 1.0, 0.0)
    plane_normal: tuple | None = None
    sweep_e1: tuple | None = None
    sweep_e2: tuple | None = None
    sweep_angle: float = 0.0
    prev_plane_angle: float | None = None
    prev_t: float | None = None
    last_dt: float = 0.1
    land_duty: float = 0.0
    touchdown_t: float | None = None
    touchdown_speed: float | None = None
    settled_t: float | None = None
    min_coast_peri: float = 0.0
    max_coast_radius: float = 0.0
    decoupler_fired: bool = False
    sep_drop_seen_t: float | None = None
    separated_t: float | None = None
    post_sep_blocks: int = 0

    def update_gm(self, frame: Frame) -> None:
        if frame.gravity_available and frame.g_mag > 1e-6 and frame.radius > 1e-6:
            sample = frame.g_mag * frame.radius * frame.radius
            if self.gm == 0.0:
                self.gm = sample
            elif frame.g_mag < 21.9:
                # |g| clamps at 22.0 near the surface, so GM = g*r^2 samples
                # are only unbiased in the unclamped region.
                self.gm = 0.9 * self.gm + 0.1 * sample

    def elements(self, frame: Frame) -> dict:
        gm = self.gm
        if gm <= 0.0:
            return {"apoapsis": 0.0, "periapsis": 0.0, "ecc": 1.0}
        h_vec = v_cross(frame.rel, frame.vel)
        h = v_norm(h_vec)
        energy = 0.5 * frame.speed * frame.speed - gm / max(frame.radius, 1e-6)
        ecc_sq = 1.0 + 2.0 * energy * h * h / (gm * gm)
        ecc = math.sqrt(max(0.0, ecc_sq))
        semi_latus = h * h / gm
        periapsis = semi_latus / (1.0 + ecc) if (1.0 + ecc) > 1e-9 else 0.0
        apoapsis = semi_latus / (1.0 - ecc) if ecc < 1.0 - 1e-9 else float("inf")
        return {"apoapsis": apoapsis, "periapsis": periapsis, "ecc": ecc, "h": h}

    def v_circular(self, radius: float) -> float:
        return math.sqrt(self.gm / max(radius, 1e-6)) if self.gm > 0.0 else 0.0

    # -- helpers -------------------------------------------------------------

    def transition(self, *, stage: str, frame: Frame, note: str = "") -> None:
        self.events.append(
            {
                "t": frame.t,
                "from": self.stage,
                "to": stage,
                "altitude": frame.radius - self.surface_radius,
                "v_tan": frame.v_tan,
                "v_rad": frame.v_rad,
                "fuel_fraction": frame.fuel_fraction,
                "note": note,
            }
        )
        print(f"[{frame.t:8.2f}s] stage {self.stage} -> {stage} "
              f"(alt={frame.radius - self.surface_radius:.1f}, v_tan={frame.v_tan:.1f}, "
              f"v_rad={frame.v_rad:.1f}, fuel={frame.fuel_fraction:.2f}) {note}",
              flush=True)
        self.stage = stage
        self.stage_started = frame.t

    def prograde_horizontal(self, frame: Frame) -> tuple:
        if frame.v_tan > 0.5:
            return v_unit(frame.v_tan_vec)
        east = self.east if self.east is not None else (1.0, 0.0, 0.0)
        horizontal = v_sub(east, v_scale(frame.up_r, v_dot(east, frame.up_r)))
        return v_unit(horizontal)

    def update_sweep(self, frame: Frame) -> None:
        if self.plane_normal is None:
            return
        e1 = self.sweep_e1
        e2 = self.sweep_e2
        angle = math.atan2(v_dot(frame.rel, e2), v_dot(frame.rel, e1))
        if self.prev_plane_angle is not None:
            delta = angle - self.prev_plane_angle
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            self.sweep_angle += delta
        self.prev_plane_angle = angle

    def h_agl(self, frame: Frame) -> float:
        return frame.radius - self.surface_radius - self.ground_clearance

    def dt_est(self, frame: Frame) -> float:
        if self.prev_t is not None and frame.t > self.prev_t:
            self.last_dt = min(0.5, frame.t - self.prev_t)
        return self.last_dt

    @staticmethod
    def slew_toward(*, current: tuple, target: tuple, max_step: float) -> tuple:
        """Rotate unit vector `current` toward `target` by at most max_step rad."""
        cos_angle = max(-1.0, min(1.0, v_dot(current, target)))
        angle = math.acos(cos_angle)
        if angle <= max_step or angle < 1e-6:
            return target
        axis = v_unit(v_cross(current, target))
        if v_norm(axis) < 1e-9:
            return target
        cos_step = math.cos(max_step)
        sin_step = math.sin(max_step)
        term1 = v_scale(current, cos_step)
        term2 = v_scale(v_cross(axis, current), sin_step)
        term3 = v_scale(axis, v_dot(axis, current) * (1.0 - cos_step))
        return v_unit(v_add(v_add(term1, term2), term3))

    def guidance(self, *, frame: Frame, elements: dict, r_t: float,
                 energy_radius: float | None = None) -> tuple[tuple, float]:
        """Velocity-vector guidance shared by guided_ascent and orbit_coast.

        Demanded accel = gravity compensation (fading with v_tan/v_circ and
        with apoapsis overshoot) + climb-rate tracking (authority-capped) +
        tangential deficit with drag feedforward.  Attitude command is the
        demanded accel direction, slew-rate limited; duty is the projection
        of the demand onto the actual thrust axis.

        ``energy_radius`` overrides the radius used to build the specific-
        energy target in the aero-locked (throttle-only) regime.  Passing a
        fixed ``peri_target`` makes the guidance hold a circular-orbit
        energy at the periapsis radius (so the ellipse stays above the S02
        gate) instead of at ``r_t``.  Keep it a FIXED radius: an r-dependent
        value would drift the energy goal upward and self-excite.
        """
        config = self.config
        if frame.v_tan > config.throttle_mode_vtan:
            # Aero-locked regime: fly at zero AoA (attitude = velocity) and
            # manage energy with the throttle only.
            desired = v_unit(frame.vel) if frame.speed > 1.0 else frame.up_r
            self.guidance_dir = desired
            energy = 0.5 * frame.speed * frame.speed - self.gm / max(frame.radius, 1e-6)
            energy_target = -self.gm / (2.0 * (energy_radius if energy_radius is not None else r_t))
            duty = (config.throttle_ff_duty
                    + config.throttle_ke * (energy_target - energy) / 1000.0
                    - config.throttle_kvr * frame.v_rad)
            duty = max(0.0, min(1.0, duty))
            if config.use_active_attitude:
                duty *= max(0.0, v_dot(frame.up_axis, desired))
            return desired, duty
        east = self.prograde_horizontal(frame)
        v_circ_local = self.v_circular(frame.radius)
        v_circ_target = self.v_circular(r_t)
        frac = min(1.0, frame.v_tan / max(v_circ_local, 1e-6))
        vr_target = min(config.ascent_vr_max,
                        max(config.ascent_vr_min, config.ascent_vr_gain * (r_t - frame.radius)))
        apo = elements["apoapsis"]
        if apo == float("inf"):
            gcomp = 0.0
        else:
            gcomp = max(0.0, min(1.0, (1.1 * r_t - apo) / max(0.1 * r_t, 1e-6)))
        # also fade gravity compensation when the climb rate overshoots
        gcomp *= max(0.0, min(1.0, 1.0 - max(0.0, frame.v_rad - vr_target) / 15.0))
        vr_term = config.ascent_vr_kp * (vr_target - frame.v_rad)
        vr_term = max(-config.ascent_vr_authority, min(config.ascent_vr_authority, vr_term))
        a_rad = frame.g_mag * gcomp * (1.0 - frac * frac) + vr_term
        a_tan = config.ascent_drag_ff + config.ascent_vt_kp * (v_circ_target - frame.v_tan)
        a_rad = max(-15.0, a_rad)
        a_tan = max(0.0, min(config.thrust_accel, a_tan))
        accel_vec = v_add(v_scale(frame.up_r, a_rad), v_scale(east, a_tan))
        a_mag = v_norm(accel_vec)
        raw_dir = v_unit(accel_vec) if a_mag > 1e-6 else frame.up_r
        self.guidance_dir = self.slew_toward(
            current=self.guidance_dir, target=raw_dir,
            max_step=math.radians(config.ascent_slew_deg_s) * self.dt_est(frame))
        a_axis = v_dot(accel_vec, frame.up_axis)
        duty = max(0.0, min(1.0, a_axis / config.full_thrust_accel))
        return self.guidance_dir, duty

    # -- the sequencer -------------------------------------------------------

    def decide(self, *, frame: Frame) -> dict:
        """Returns {"duty", "desired_dir", "decoupler", ...debug fields}."""
        config = self.config
        self.update_gm(frame)
        if self.surface_radius:
            self.max_altitude = max(
                self.max_altitude, frame.radius - self.surface_radius
            )
        elements = self.elements(frame)
        r_t = self.surface_radius + config.target_altitude if self.surface_radius else 0.0
        duty = 0.0
        desired = frame.up_r
        decoupler = None
        energy = 0.0
        energy_target = 0.0

        if self.stage == "pad":
            if frame.t >= 0.4 and frame.gravity_available:
                self.surface_radius = frame.surface_radius
                self.pad_radius = frame.radius
                self.ground_clearance = frame.radius - frame.surface_radius
                self.initial_blocks = frame.block_count
                # Tilt in the plane controllable by wheel A (machine +x at
                # pad, projected horizontal); world == machine frame at spawn.
                east_raw = q_rotate(frame.rot, (1.0, 0.0, 0.0))
                east = v_sub(east_raw, v_scale(frame.up_r, v_dot(east_raw, frame.up_r)))
                self.east = v_unit(east)
                self.guidance_dir = frame.up_r
                self.transition(
                    stage="guided_ascent",
                    frame=frame,
                    note=(f"GM={self.gm:.0f} surface_r={self.surface_radius:.1f} "
                          f"clearance={self.ground_clearance:.1f} g={frame.g_mag:.2f} "
                          f"blocks={frame.block_count}"),
                )
            return {"duty": 0.0, "desired_dir": desired, "decoupler": None,
                    "apoapsis": elements["apoapsis"],
                    "periapsis": elements["periapsis"], "ecc": elements["ecc"],
                    "sweep_deg": math.degrees(self.sweep_angle), "gm": self.gm,
                    "h_agl": 0.0, "energy": 0.0, "energy_target": 0.0,
                    "r_raise": config.raise_target_radius,
                    "peri_target": config.s02_peri_threshold}

        if self.stage == "guided_ascent":
            desired, duty = self.guidance(frame=frame, elements=elements, r_t=r_t)
            apo = elements["apoapsis"]
            v_circ_local = self.v_circular(frame.radius)
            # S02 insertion gate: the ellipse must clear the 1100 periapsis
            # radius (planet-center distance), not just the ground.  The
            # margin (peri >= 1130) is mandatory for EVERY insertion path:
            # the previous flight entered on powered_ok at peri=991.6 and
            # min_coast_peri then never recovered for the full 3-lap coast.
            insertion_peri = config.s02_peri_threshold + config.s02_peri_margin
            peri_ok = (
                elements["periapsis"] >= insertion_peri
            )
            apo_ok = (
                apo != float("inf")
                and self.surface_radius + config.orbit_apo_frac * config.target_altitude
                <= apo <= self.surface_radius + 2.5 * config.target_altitude
            )
            circ_ok = (
                frame.v_tan >= v_circ_local
                and abs(frame.v_rad) < 4.0
                and frame.radius >= config.s02_radius_threshold
                and elements["periapsis"] >= insertion_peri
            )
            energy = 0.5 * frame.speed * frame.speed - self.gm / max(frame.radius, 1e-6)
            energy_target = -self.gm / (2.0 * r_t)
            powered_ok = (
                frame.v_tan > config.throttle_mode_vtan
                and abs(energy - energy_target) < 1200.0
                and abs(frame.v_rad) < 6.0
                and frame.radius >= config.s02_radius_threshold
                and elements["periapsis"] >= insertion_peri
            )
            if (peri_ok and apo_ok) or circ_ok or powered_ok:
                normal = v_unit(v_cross(frame.rel, frame.vel))
                self.plane_normal = normal
                self.sweep_e1 = v_unit(frame.rel)
                self.sweep_e2 = v_unit(v_cross(normal, self.sweep_e1))
                self.sweep_angle = 0.0
                self.prev_plane_angle = None
                self.min_coast_peri = elements["periapsis"]
                self.max_coast_radius = frame.radius
                self.transition(
                    stage="orbit_coast", frame=frame,
                    note=(f"ecc={elements['ecc']:.3f} "
                          f"peri={elements['periapsis'] - self.surface_radius:.1f} "
                          f"apo={elements['apoapsis'] - self.surface_radius:.1f}"),
                )

        elif self.stage == "orbit_coast":
            # Same guidance law as the ascent: hold a stable powered
            # near-circular orbit against the heavy drag.  The target is a
            # *periapsis radius* (coast_radius = 1120, above the 1100 S02
            # gate) with headroom for fuel economy: the energy target is
            # -GM/(2*peri_target) (circular-orbit energy at the periapsis
            # radius), so the guidance keeps the ellipse from collapsing
            # back below 1100 while still leaving fuel for the deorbit and
            # landing.  NOTE: energy_radius must be a FIXED target radius;
            # using r + peri_target would let the energy goal drift upward
            # with the current radius and self-excite into an escape.
            self.update_sweep(frame)
            peri = elements["periapsis"]
            self.min_coast_peri = min(self.min_coast_peri, peri)
            self.max_coast_radius = max(self.max_coast_radius, frame.radius)
            swept = abs(math.degrees(self.sweep_angle))
            # If drag pulled the periapsis near the gate, lift the target
            # back to the insertion periapsis early (before it can dip
            # below 1100 and fail the orbit gate).
            if peri < config.s02_peri_threshold + 20.0:
                peri_target = config.raise_target_radius
            else:
                peri_target = config.coast_radius
            desired, duty = self.guidance(
                frame=frame, elements=elements, r_t=peri_target,
                energy_radius=peri_target,
            )
            if swept >= config.orbit_sweep_deg:
                self.events.append(
                    {"t": frame.t, "event": "three_laps_complete",
                     "sweep_deg": swept, "radius": frame.radius,
                     "v_tan": frame.v_tan, "fuel_fraction": frame.fuel_fraction,
                     "min_coast_peri": self.min_coast_peri}
                )
                print(f"[{frame.t:8.2f}s] 3 laps complete "
                      f"(sweep={swept:.0f}deg, r={frame.radius:.1f}, "
                      f"min_peri={self.min_coast_peri:.1f}, "
                      f"fuel={frame.fuel_fraction:.2f})", flush=True)
                if config.separation_enabled:
                    self.transition(stage="separate", frame=frame,
                                    note=f"swept={swept:.0f}deg")
                else:
                    self.transition(stage="kill_burn", frame=frame,
                                    note=f"swept={swept:.0f}deg")
            elif (config.coast_early_exit_fuel > 0.0
                    and frame.fuel_fraction <= config.coast_early_exit_fuel
                    and swept >= 360.0):
                # Fuel-saving bail: one full lap is banked but the tank is
                # low; cut the coast short so the landing legs have fuel.
                self.events.append(
                    {"t": frame.t, "event": "coast_early_exit",
                     "sweep_deg": swept, "fuel_fraction": frame.fuel_fraction}
                )
                print(f"[{frame.t:8.2f}s] coast early exit (fuel guard): "
                      f"swept={swept:.0f}deg fuel={frame.fuel_fraction:.2f}",
                      flush=True)
                self.transition(stage="kill_burn", frame=frame,
                                note=f"fuel_guard swept={swept:.0f}deg")

        elif self.stage == "separate":
            # Fire the Decoupler once, then wait for the block-count drop.
            # duty stays 0 from here on: the payload must never be pushed.
            self.update_sweep(frame)
            if not self.decoupler_fired:
                decoupler = (DECOUPLER_GUID, DECOUPLER_SEPARATION_KEY)
                self.decoupler_fired = True
                self.events.append(
                    {"t": frame.t, "event": "decoupler_fired",
                     "blocks_before": frame.block_count}
                )
                print(f"[{frame.t:8.2f}s] Decoupler fired (blocks={frame.block_count})",
                      flush=True)
            if frame.block_count <= self.initial_blocks - config.separation_block_drop:
                if self.sep_drop_seen_t is None:
                    self.sep_drop_seen_t = frame.t
                elif frame.t - self.sep_drop_seen_t >= 0.5:
                    self.separated_t = frame.t
                    self.post_sep_blocks = frame.block_count
                    self.transition(
                        stage="payload_coast", frame=frame,
                        note=f"separated blocks={frame.block_count}",
                    )
            elif frame.t - self.stage_started > config.separation_timeout:
                self.abort_reason = "separation_failed"
                self.done = True

        elif self.stage == "payload_coast":
            # Purely ballistic: duty is ALWAYS 0.  Sweep keeps accumulating
            # from insertion until 3 full laps are done.
            self.update_sweep(frame)
            desired = frame.up_r
            duty = 0.0
            if abs(math.degrees(self.sweep_angle)) >= config.orbit_sweep_deg:
                self.events.append(
                    {"t": frame.t, "event": "three_laps_complete",
                     "sweep_deg": math.degrees(self.sweep_angle),
                     "radius": frame.radius, "v_tan": frame.v_tan}
                )
                print(f"[{frame.t:8.2f}s] 3 laps complete (payload) "
                      f"(sweep={math.degrees(self.sweep_angle):.0f}deg)",
                      flush=True)
                self.done = True

        elif self.stage == "kill_burn":
            # Deorbit braking: thrust along -v_tan ONLY (retrograde).  The
            # old hover-brake law (radial hold a_up + 30 deg tilt) put a
            # large radial component into the thrust at orbital speed; it
            # pumped energy into the orbit (v_rad 0 -> +102) and climbed
            # straight into escaped_altitude_bound (observed at alt 2204.9).
            # At 2200+ alt the local escape speed is ~78 u/s, so that abort
            # was a genuine escape, not a false positive.  Keep the throttle
            # shut until the body is nearly aligned with the retro direction:
            # early thrust along the coast attitude accelerates the orbit.
            retro_dir = (
                v_scale(v_unit(frame.v_tan_vec), -1.0)
                if frame.v_tan > 0.5 else (0.0, 0.0, 0.0)
            )
            desired = retro_dir
            align = v_dot(frame.up_axis, desired)
            a_retro = min(
                config.thrust_accel,
                config.kill_burn_kv * max(0.0, frame.v_tan - config.kill_burn_exit_vtan),
            )
            a_retro = max(0.0, a_retro)
            if align > config.kill_burn_align_gate:
                duty = max(0.0, min(1.0, a_retro / config.full_thrust_accel))
            elif align > 0.5:
                # 30% pilot thrust while the attitude is still turning so the
                # body keeps rotating through; mostly just gravity does the
                # radial work here.
                duty = 0.3 * max(0.0, min(1.0, a_retro / config.full_thrust_accel))
            else:
                duty = 0.0
            if frame.v_tan < config.kill_burn_exit_vtan or self.h_agl(frame) < 6.0:
                self.transition(stage="final_descent", frame=frame,
                                note=f"speed={frame.speed:.1f} h_agl={self.h_agl(frame):.1f}")

        elif self.stage == "final_descent":
            h_agl = self.h_agl(frame)
            v_hor_vec = frame.v_tan_vec
            v_hor = v_norm(v_hor_vec)
            # gentle tilt only while high enough; near the ground hold pure
            # vertical (the tilt chase near the ground flip-flopped faster
            # than the wheels could follow and PUMPED horizontal speed).
            if h_agl > 8.0 and v_hor > 2.0:
                tilt = min(math.radians(config.tilt_max_deg), config.tilt_gain * v_hor)
                tilt_dir = v_scale(v_unit(v_hor_vec), -1.0)
                raw_desired = v_unit(v_add(v_scale(frame.up_r, math.cos(tilt)), v_scale(tilt_dir, math.sin(tilt))))
            else:
                raw_desired = frame.up_r
            self.guidance_dir = self.slew_toward(
                current=self.guidance_dir, target=raw_desired,
                max_step=math.radians(12.0) * self.dt_est(frame))
            desired = self.guidance_dir
            v_des = -max(config.final_min_sink, min(25.0, config.final_descent_gain * h_agl))
            # Adaptive (integrating) throttle: the craft is much lighter than
            # at launch, so the calibrated hover duty actually CLIMBS.  The
            # integrator finds the true hover duty by itself.
            err = v_des - frame.v_rad
            self.land_duty += 0.06 * err * self.dt_est(frame)
            self.land_duty = max(0.0, min(1.0, self.land_duty))
            duty = max(0.0, min(1.0, self.land_duty + 0.05 * err))
            if h_agl <= 0.6 and abs(frame.v_rad) <= config.final_touchdown_speed + 1.0:
                self.touchdown_t = frame.t
                self.touchdown_speed = frame.speed
                self.transition(stage="touchdown", frame=frame,
                                note=f"touchdown speed={frame.speed:.2f} "
                                     f"integrity={frame.machine_integrity:.2f}")

        elif self.stage == "touchdown":
            # S02 recovery gate: engines stay off and the vehicle must remain
            # settled (total speed <= 0.5) for settled_hold_sec = 10 s.
            duty = 0.0
            desired = frame.up_r
            if frame.speed > 0.5:
                self.settled_t = None
            elif self.settled_t is None:
                self.settled_t = frame.t
            if self.settled_t is not None and frame.t - self.settled_t >= config.settled_hold_sec:
                integrity_ok = frame.machine_integrity >= config.required_integrity
                self.events.append(
                    {"t": frame.t, "event": "settled_hold_complete",
                     "hold_sec": config.settled_hold_sec,
                     "integrity": frame.machine_integrity,
                     "integrity_ok": integrity_ok,
                     "speed": frame.speed}
                )
                print(f"[{frame.t:8.2f}s] settled hold complete "
                      f"({config.settled_hold_sec}s, "
                      f"integrity={frame.machine_integrity:.2f}, "
                      f"ok={integrity_ok})", flush=True)
                self.done = True

        # global aborts
        if frame.t > config.max_duration:
            self.abort_reason = "max_duration"
            self.done = True
        if self.initial_blocks and self.stage not in ("pad", "separate"):
            baseline = self.post_sep_blocks if self.stage == "payload_coast" else self.initial_blocks
            if baseline and frame.block_count < baseline:
                self.abort_reason = f"crash_blocks_lost_{baseline - frame.block_count}"
                self.done = True
        # Escape boundary: LONE ORB is finite (surface 600, GM 8.4568e6), so
        # beyond ~2200 km altitude the local escape speed drops below 80 u/s
        # and a burn there bleeds straight out of the sandbox.  The value is
        # max(4*target_altitude, raise_target_radius+400) = 2200 alt; the
        # failed flight crossed it at speed ~103 with escape speed ~78, i.e.
        # a genuine escape (fixed by the retrograde kill_burn, not by
        # loosening this bound).
        if self.surface_radius and frame.radius > self.surface_radius + max(
            4.0 * config.target_altitude, config.raise_target_radius + 400.0
        ):
            self.abort_reason = "escaped_altitude_bound"
            self.done = True
        if (self.stage in ("orbit_coast", "kill_burn", "payload_coast")
                and self.h_agl(frame) < 1.5 and frame.speed < 2.0):
            self.abort_reason = "unexpected_ground_contact"
            self.done = True
        if (self.stage not in ("pad", "touchdown", "payload_coast")
                and frame.fuel_fraction <= 0.005):
            self.abort_reason = "fuel_exhausted"
            self.done = True

        self.prev_t = frame.t
        return {
            "duty": duty,
            "desired_dir": desired,
            "decoupler": decoupler,
            "apoapsis": elements["apoapsis"],
            "periapsis": elements["periapsis"],
            "ecc": elements["ecc"],
            "sweep_deg": math.degrees(self.sweep_angle),
            "gm": self.gm,
            "h_agl": self.h_agl(frame) if self.surface_radius else 0.0,
            "energy": energy,
            "energy_target": energy_target,
            "r_raise": config.raise_target_radius,
            "peri_target": config.s02_peri_threshold,
        }


# ---------------------------------------------------------------------------
# Calibration policy (open-loop pulses; re-fit thrust_accel and wheel axes)
# ---------------------------------------------------------------------------

def _list_blocks(blocks) -> str:
    return ", ".join(
        f"{block.local_index}:{block.block_id}:{block.name}" for block in blocks
    )


def discover_hardware() -> None:
    """Bind root / engine / wheel / decoupler GUIDs using the same names
    inspect-machine prints.  Decoupler detection is optional: a machine with
    no Decoupler is treated as a single stage (separation disabled)."""
    global ROOT_GUID, ENGINE_GUIDS, WHEEL_GUIDS, DECOUPLER_GUID
    bsg_value = os.environ.get(ENV_MACHINE_BSG, "")
    catalog_value = os.environ.get(ENV_CHANNEL_CATALOG, "")
    if bsg_value.strip() == "" or catalog_value.strip() == "":
        raise RuntimeError(
            f"{ENV_MACHINE_BSG} and {ENV_CHANNEL_CATALOG} must be set so this "
            "controller can discover GUIDs on the rebuilt machine."
        )
    from besiege_cli.machine import blocks_named, parse_bsg

    _, blocks, _ = parse_bsg(bsg_value, catalog_path=catalog_value)
    starting = [block for block in blocks_named(blocks, "Starting Block") if block.guid]
    boosters = [block for block in blocks_named(blocks, "Booster") if block.guid]
    wheels = [block for block in blocks_named(blocks, "Reaction Steering Block") if block.guid]
    decouplers = [block for block in blocks_named(blocks, "Decoupler") if block.guid]
    inventory = _list_blocks(blocks)
    if len(starting) != 1:
        raise RuntimeError(
            f"Expected exactly one Starting Block with a GUID, found {len(starting)}. "
            f"Blocks: {inventory}"
        )
    if len(boosters) < 4:
        raise RuntimeError(
            f"Expected at least 4 Booster blocks, found {len(boosters)}. "
            f"Blocks: {inventory}"
        )
    if len(wheels) != 2:
        raise RuntimeError(
            f"Expected exactly 2 Reaction Steering Blocks, found {len(wheels)}. "
            f"Blocks: {inventory}"
        )
    ROOT_GUID = starting[0].guid
    ENGINE_GUIDS = tuple(block.guid for block in boosters)
    WHEEL_GUIDS = tuple(block.guid for block in wheels)
    DECOUPLER_GUID = decouplers[0].guid if decouplers else ""
    print(
        f"Discovered hardware: root={ROOT_GUID} "
        f"engines={len(ENGINE_GUIDS)} wheels={len(WHEEL_GUIDS)} "
        f"decoupler={DECOUPLER_GUID or '(none: single stage)'}",
        flush=True,
    )


def bind_wheel_axes(*, env: ControllerClient, sample, root_record) -> None:
    """Classify wheels from the first telemetry frame.

    LeftKey torque axis = -(machine-frame offset) unit vector by default
    (the wheel normal).  machine.json does not encode the mounting flip, so
    the live axes must be confirmed once (HEAVY_LAUNCHER_POLICY=calibrate +
    analyze_calibration); if they diverge, hard-code WHEEL_CALIBRATION_OVERRIDE.
    """
    global WHEEL_A_GUID, WHEEL_B_GUID, WHEEL_CALIBRATION
    root_pos = tuple(float(value) for value in root_record["position"])
    root_rot = tuple(float(value) for value in root_record["rotation"])
    wanted = {guid.lower() for guid in WHEEL_GUIDS}
    offsets: list[tuple[str, tuple[float, float, float]]] = []
    for record in sample.records:
        guid = env.guid_of(int(record["target_index"]))
        if guid.lower() not in wanted:
            continue
        pos = tuple(float(value) for value in record["position"])
        rel_machine = q_rotate(q_conj(root_rot), v_sub(pos, root_pos))
        offsets.append((guid, rel_machine))
        print(
            f"  wheel {guid} machine-frame offset="
            f"({rel_machine[0]:+.2f}, {rel_machine[1]:+.2f}, {rel_machine[2]:+.2f})",
            flush=True,
        )
    if len(offsets) != 2:
        raise RuntimeError(
            f"First BAT4 sample is missing Reaction Steering Blocks; found {len(offsets)}."
        )
    first, second = offsets
    if abs(first[1][0]) >= abs(second[1][0]):
        wheel_a, wheel_b = first[0], second[0]
    else:
        wheel_a, wheel_b = second[0], first[0]
    WHEEL_A_GUID = wheel_a
    WHEEL_B_GUID = wheel_b
    if WHEEL_CALIBRATION_OVERRIDE:
        WHEEL_CALIBRATION = {
            WHEEL_A_GUID: {"axis": WHEEL_CALIBRATION_OVERRIDE.get(WHEEL_A_GUID) or v_unit(v_scale(first[1], -1.0))},
            WHEEL_B_GUID: {"axis": WHEEL_CALIBRATION_OVERRIDE.get(WHEEL_B_GUID) or v_unit(v_scale(second[1], -1.0))},
        }
    else:
        WHEEL_CALIBRATION = {
            WHEEL_A_GUID: {"axis": v_unit(v_scale(first[1], -1.0))},
            WHEEL_B_GUID: {"axis": v_unit(v_scale(second[1], -1.0))},
        }
    print(f"Wheel A={WHEEL_A_GUID} axis={WHEEL_CALIBRATION[WHEEL_A_GUID]['axis']}", flush=True)
    print(f"Wheel B={WHEEL_B_GUID} axis={WHEEL_CALIBRATION[WHEEL_B_GUID]['axis']}", flush=True)


class CalibratePolicy:
    """Scripted pulses: boost airborne first, then pulse each wheel in free
    fall.  Booster pulses are used to re-fit MissionConfig.thrust_accel;
    wheel pulses confirm the LeftKey torque axes assumed by bind_wheel_axes
    (the delta-omega axis in analyze_calibration is the true torque axis).
    """

    ENGINE_WINDOW = (1.0, 3.5)
    WHEEL_A_WINDOW = (4.5, 5.5)
    WHEEL_B_WINDOW = (7.0, 8.0)

    def __init__(self):
        self.duration = 9.5

    def decide(self, *, frame: Frame) -> list[tuple[str, str]]:
        schedule = (
            (*self.ENGINE_WINDOW, [(guid, "ThrustKey") for guid in ENGINE_GUIDS]),
            (*self.WHEEL_A_WINDOW, [(WHEEL_A_GUID, "LeftKey")]),
            (*self.WHEEL_B_WINDOW, [(WHEEL_B_GUID, "LeftKey")]),
        )
        controls: list[tuple[str, str]] = []
        for start, end, channels in schedule:
            if start <= frame.t < end:
                controls.extend(channels)
        return controls


# ---------------------------------------------------------------------------
# Episode loop
# ---------------------------------------------------------------------------

def run_episode(
    *,
    env: ControllerClient,
    mission: MissionController | None,
    calibrate: CalibratePolicy | None,
    attitude: AttitudeController,
    duration: float,
    timeout: float,
    steps_path: Path,
) -> list[dict]:
    env.arm()
    sample = env.wait_until_running(timeout=timeout)
    if env._block_guids is None:
        env.load_block_table()
    print(f"sliders={len(env.sliders)}", flush=True)
    for slider in env.sliders:
        print(
            f"  slider {slider.block_guid} {slider.name} "
            f"[{slider.minimum},{slider.maximum}] default={slider.default}",
            flush=True,
        )

    power_sliders = discover_booster_power_sliders(env=env)
    wheel_sliders = discover_wheel_speed_sliders(env=env)
    pin_operating_sliders(
        env=env,
        assignments=[
            *((slider, slider.maximum) for slider in power_sliders),
            *((slider, slider.default) for slider in wheel_sliders),
        ],
    )

    # Resolve the Decoupler separation key from the live channel catalog
    # (only needed when the machine actually has one).
    decoupler_channel = None
    if DECOUPLER_GUID:
        try:
            for channel in env.channels:
                if (channel.block_guid.lower() == DECOUPLER_GUID.lower()
                        and channel.name == DECOUPLER_SEPARATION_KEY):
                    decoupler_channel = channel
                    break
        except Exception as exc:  # pragma: no cover - defensive
            print(f"WARNING: could not resolve Decoupler channel: {exc}", flush=True)
        if decoupler_channel is None:
            raise RuntimeError(
                f"Decoupler {DECOUPLER_GUID} does not publish "
                f"{DECOUPLER_SEPARATION_KEY!r}; cannot trigger separation."
            )
        print(
            f"Decoupler channel: {decoupler_channel.block_guid} "
            f"{decoupler_channel.name} (keylist={decoupler_channel.keylist_index})",
            flush=True,
        )

    print(
        "Guidance model: thrust_accel="
        f"{mission.config.thrust_accel if mission is not None else 36.0} "
        f"full_thrust_accel="
        f"{mission.config.full_thrust_accel if mission is not None else 36.0} "
        f"use_active_attitude="
        f"{mission.config.use_active_attitude if mission is not None else True} "
        f"separation_enabled="
        f"{mission.config.separation_enabled if mission is not None else False}",
        flush=True,
    )

    throttle = ThrottlePwm()
    steps: list[dict] = []
    last_report = 0.0

    # Stream each step to disk immediately so a crash/interrupt never loses data.
    with steps_path.open("w", encoding="utf-8") as steps_handle:
        while True:
            root_record = None
            for record in sample.records:
                target_guid = env.guid_of(int(record["target_index"]))
                if target_guid.lower() == ROOT_GUID.lower():
                    root_record = record
                    break
            if root_record is None:
                raise ValueError(
                    f"BAT4 sample does not include root {ROOT_GUID}."
                )
            if not WHEEL_CALIBRATION:
                bind_wheel_axes(env=env, sample=sample, root_record=root_record)
            frame = Frame.from_telemetry(sample, root_record)

            if calibrate is not None:
                controls = calibrate.decide(frame=frame)
                debug = {}
                decision = {"duty": 0.0}
                fired = any(channel == "ThrustKey" for _guid, channel in controls)
                if frame.t >= calibrate.duration:
                    env.send_channels(channels=[])
                    break
            else:
                assert mission is not None
                decision = mission.decide(frame=frame)
                if mission.config.use_active_attitude:
                    controls, debug = attitude.commands(
                        frame=frame, desired_dir=decision["desired_dir"]
                    )
                else:
                    controls, debug = [], {"att_err": 0.0, "align": 0.0}
                fired = throttle.fire(duty=decision["duty"])
                if fired:
                    controls = controls + [
                        (guid, "ThrustKey") for guid in ENGINE_GUIDS
                    ]
                dec = decision.get("decoupler")
                if dec:
                    controls.append((dec[0], dec[1]))
                # HARD GUARANTEE: after separation the payload is never pushed.
                if mission.stage == "payload_coast" and any(
                    channel == "ThrustKey" for _guid, channel in controls
                ):
                    raise RuntimeError(
                        "propulsive command after payload separation"
                    )
                if mission.done:
                    env.send_channels(channels=[])
                    break

            env.send_channels(channels=controls)
            row = {
                "t": frame.t,
                "stage": mission.stage if mission is not None else "calibrate",
                "pos": frame.pos,
                "vel": frame.vel,
                "rot": frame.rot,
                "ang_vel": frame.ang_vel,
                "gravity_acc": frame.gravity_acc,
                "radius": frame.radius,
                "altitude": frame.radius - frame.surface_radius,
                "v_rad": frame.v_rad,
                "v_tan": frame.v_tan,
                "speed": frame.speed,
                "g_mag": frame.g_mag,
                "surface_radius": frame.surface_radius,
                "fuel_fraction": frame.fuel_fraction,
                "block_count": frame.block_count,
                "machine_integrity": frame.machine_integrity,
                "duty": decision.get("duty", 0.0),
                "engines_fired": fired,
                "controls": [f"{guid[:8]}:{channel}" for guid, channel in controls],
                **{key: value for key, value in decision.items()
                   if key in ("apoapsis", "periapsis", "ecc", "sweep_deg", "gm", "h_agl")},
                **debug,
            }
            steps.append(row)
            steps_handle.write(json.dumps(row) + "\n")
            steps_handle.flush()
            if frame.t - last_report >= 5.0:
                last_report = frame.t
                stage = mission.stage if mission is not None else "calibrate"
                print(
                    f"  t={frame.t:7.1f}s stage={stage:13s} "
                    f"alt={frame.radius - frame.surface_radius:8.1f} "
                    f"v_tan={frame.v_tan:6.1f} v_rad={frame.v_rad:6.1f} "
                    f"fuel={frame.fuel_fraction:.2f} "
                    f"att={debug.get('att_err', 0):.2f} "
                    f"align={debug.get('align', 0):.2f} "
                    f"integrity={frame.machine_integrity:.2f}",
                    flush=True,
                )
            if frame.t >= duration:
                print("Duration limit reached; stopping.")
                break
            sample = env.next_sample(timeout=2.0)

    return steps


# ---------------------------------------------------------------------------
# Calibration analysis
# ---------------------------------------------------------------------------

def analyze_calibration(*, steps: list[dict]) -> dict:
    def window(t0: float, t1: float) -> list[dict]:
        return [row for row in steps if t0 <= row["t"] < t1]

    result: dict = {}
    for label, (t0, t1) in (
        ("wheel_a_left", CalibratePolicy.WHEEL_A_WINDOW),
        ("wheel_b_left", CalibratePolicy.WHEEL_B_WINDOW),
    ):
        rows = window(t0 + 0.3, t1)
        if len(rows) < 3:
            result[label] = {"error": "insufficient samples"}
            continue
        first, last = rows[0], rows[-1]
        delta_omega_world = v_sub(last["ang_vel"], first["ang_vel"])
        # express in machine frame using the rotation at the window center
        mid = rows[len(rows) // 2]
        delta_machine = q_rotate(q_conj(mid["rot"]), delta_omega_world)
        dt = last["t"] - first["t"]
        result[label] = {
            "delta_omega_machine": [round(value, 4) for value in delta_machine],
            "alpha_machine": [round(value / dt, 4) for value in delta_machine],
            "axis_unit": [round(value, 3) for value in v_unit(delta_machine)],
            "dt": round(dt, 2),
        }
    engine_t0, engine_t1 = CalibratePolicy.ENGINE_WINDOW
    rows = window(engine_t0 + 0.4, engine_t1 - 0.1)
    if len(rows) >= 3:
        first, last = rows[0], rows[-1]
        dt = last["t"] - first["t"]
        dv = v_sub(last["vel"], first["vel"])
        g_avg = v_scale(v_add(first["gravity_acc"], last["gravity_acc"]), 0.5)
        thrust_acc = v_sub(v_scale(dv, 1.0 / dt), g_avg)
        result["engines"] = {
            "thrust_accel_world": [round(value, 3) for value in thrust_acc],
            "thrust_accel_mag": round(v_norm(thrust_acc), 3),
            "fuel_burn_per_s": round((first["fuel_fraction"] - last["fuel_fraction"]) / dt, 5),
            "dt": round(dt, 2),
        }
    gravity_samples = [
        {"r": round(row["radius"], 2), "g": round(row["g_mag"], 4),
         "gm": round(row["g_mag"] * row["radius"] ** 2, 1)}
        for row in steps[:: max(1, len(steps) // 25)] if row["g_mag"] > 1e-6
    ]
    result["gravity_profile"] = gravity_samples
    return result


# ---------------------------------------------------------------------------
# Managed controller entry point
# ---------------------------------------------------------------------------

def main() -> int:
    run_dir_value = os.environ.get(ENV_RUN_DIR)
    if not run_dir_value:
        raise RuntimeError(
            "This controller must run under `python -m besiege_cli run`; "
            f"{ENV_RUN_DIR} is not set."
        )
    run_dir = Path(run_dir_value)
    policy = os.environ.get("HEAVY_LAUNCHER_POLICY", "mission")
    if policy not in {"mission", "calibrate"}:
        raise ValueError(
            "HEAVY_LAUNCHER_POLICY must be 'mission' or 'calibrate'."
        )

    discover_hardware()
    config = MissionConfig()
    # Auto-arm the separation branch only when a Decoupler was discovered.
    config.separation_enabled = bool(DECOUPLER_GUID)
    attitude = AttitudeController(
        kp=config.att_kp,
        kd=config.att_kd,
        deadband=config.att_deadband,
        lookahead=config.att_lookahead,
    )
    mission = MissionController(config=config) if policy == "mission" else None
    calibrate = CalibratePolicy() if policy == "calibrate" else None
    duration = config.max_duration if mission is not None else calibrate.duration + 2.0
    client = ControllerClient.from_environment(poll_interval=0.01)

    try:
        steps = run_episode(
            env=client,
            mission=mission,
            calibrate=calibrate,
            attitude=attitude,
            duration=duration,
            timeout=90.0,
            steps_path=run_dir / "steps.jsonl",
        )
    finally:
        client.close()

    summary: dict = {
        "schema": "buildarena.s02_rocket_orbit_return.v1",
        "policy": policy,
        "config": {key: getattr(config, key) for key in vars(config)},
        "run_id": client.run_id,
        "telemetry": "BuildArena ToolKit v4 BAT4 full profile at 10 Hz",
        "environment_model": {
            "gravity_center": LONE_ORB_CENTER,
            "surface_radius": LONE_ORB_SURFACE_RADIUS,
            "gm": LONE_ORB_GM,
            "max_gravity": LONE_ORB_MAX_GRAVITY,
        },
        "hardware": {
            "root_guid": ROOT_GUID,
            "engine_guids": list(ENGINE_GUIDS),
            "wheel_a_guid": WHEEL_A_GUID,
            "wheel_b_guid": WHEEL_B_GUID,
            "decoupler_guid": DECOUPLER_GUID,
        },
        "steps": len(steps),
    }
    if mission is not None:
        summary.update(
            {
                "events": mission.events,
                "final_stage": mission.stage,
                "done": mission.done,
                "abort_reason": mission.abort_reason,
                "gm_estimate": mission.gm,
                "surface_radius": mission.surface_radius,
                "max_altitude": mission.max_altitude,
                "sweep_deg": math.degrees(mission.sweep_angle),
                "min_coast_peri": mission.min_coast_peri,
                "max_coast_radius": mission.max_coast_radius,
                "orbit_gate_passed": (
                    mission.min_coast_peri >= config.s02_peri_threshold
                    and math.degrees(mission.sweep_angle) >= config.orbit_sweep_deg - 1.0
                ),
                "touchdown_t": mission.touchdown_t,
                "touchdown_speed": mission.touchdown_speed,
                "settled_t": mission.settled_t,
                "settled_hold_reached": (
                    mission.settled_t is not None
                    and mission.stage == "touchdown"
                    and (steps[-1]["t"] - mission.settled_t) >= config.settled_hold_sec - 0.5
                ),
                "recovery_ok": (
                    mission.stage == "touchdown"
                    and steps[-1]["machine_integrity"] >= config.required_integrity
                    and mission.settled_t is not None
                    and (steps[-1]["t"] - mission.settled_t) >= config.settled_hold_sec - 0.5
                ),
                "final_fuel_fraction": steps[-1]["fuel_fraction"] if steps else None,
                "final_block_count": steps[-1]["block_count"] if steps else None,
                "final_machine_integrity": (
                    steps[-1]["machine_integrity"] if steps else None
                ),
            }
        )
    else:
        assert calibrate is not None
        summary["calibration"] = analyze_calibration(steps=steps)

    (run_dir / "mission_summary.json").write_text(
        json.dumps(summary, indent=1), encoding="utf-8"
    )
    printable = {
        key: value for key, value in summary.items() if key not in ("config", "events")
    }
    print(json.dumps(printable, indent=1, default=str), flush=True)

    require_success = os.environ.get("HEAVY_LAUNCHER_REQUIRE_SUCCESS", "").strip() == "1"
    if mission is not None and mission.stage == "pad":
        raise RuntimeError(
            "Mission never left the pad; ignition / guidance did not start."
        )
    if require_success and mission is not None and (mission.abort_reason or not mission.done):
        raise RuntimeError(
            f"Mission did not complete successfully: stage={mission.stage}, "
            f"abort_reason={mission.abort_reason!r}."
        )
    if mission is not None and (mission.abort_reason or not mission.done):
        print(
            f"Mission ended without a completed landing: stage={mission.stage}, "
            f"abort_reason={mission.abort_reason!r}. "
            "Setup still accepts this as a live closed-loop demo.",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
