"""
The car as a linear model: transfer functions and state space (SYSC 3600's terms).

No ROS imports; numpy and scipy.signal only (tests: test/test_vehicle_model.py; the
tables and figures: `pixi run vehicle-model`; the page: docs/vehicle-model.md).
Nothing here drives the car: it is for tuning, for checking the delays, and for the report.

Three parts, each linear about one operating point:

  | Part         | Input                  | Output         | Order | Model                        |
  |--------------|------------------------|----------------|-------|------------------------------|
  | speed        | throttle u (0..1)      | speed v (m/s)  | 2     | throttle lag, then the car   |
  | steering     | steering command (rad) | wheel angle    | 2     | the Jrk motor, then linkage  |
  | lateral      | steering command (rad) | path errors    | 3 / 4 | bicycle, about driving       |
  |              |                        |                |       | straight at speed v          |

Speed, from the actuator model (aav_gazebo actuator_model.py) with the deadband left out:

    tau_t * du_a/dt + u_a = u           the throttle's lag (vehicle.yaml throttle_lag)
    dv/dt = a_max * u_a - d * v         full throttle gives a_max; drag d (1/s)
    =>  tau_v * dv/dt + v = K * u_a,    K = a_max / d,  tau_v = 1 / d
    P(s) = K / ((tau_v s + 1)(tau_t s + 1))

The spine's speed loop (spine-core speed_loop.rs) is a PI on the speed error, throttle side,
with a setpoint weight b (1 today; the 'smooth' profile's 0.75, TODO 27):

    u = kp (b r - v) + ki/s (r - v)
    C(s) = kp + ki / s                  the loop (what stability and the margins see)
    C_r(s) = b kp + ki / s              the path from the set speed (where the zero is)

Steering: the Jrk turns the motor toward its target at up to steer_rate (a rate limit:
nonlinear), and the linkage follows with a first-order lag, steer_lag. For a step of
size A the rate limit takes A / steer_rate; its mean delay, A / (2 steer_rate), is the
time constant of the lag that matches it. Two lags in a row are a second-order system:

    G_s(s) = 1 / ((tau_m s + 1)(tau_l s + 1)) = w_n^2 / (s^2 + 2 zeta w_n s + w_n^2)
    w_n = 1 / sqrt(tau_m tau_l),  zeta = (tau_m + tau_l) / (2 sqrt(tau_m tau_l))  (>= 1)

Two real lags can't overshoot (zeta >= 1). A Jrk whose own loop overshoots has zeta < 1:
measure the overshoot in test 3.11 and pass steer_overshoot to with_measured.

Lateral, the kinematic bicycle about the rear axle (base_link), path along x:

    x' = v cos(theta),  y' = v sin(theta),  theta' = v tan(delta) / L
    linearised about y = theta = delta = 0, with the steering's lag tau_s:
    state [e_y, e_theta, delta], input delta_cmd
    A = [[0, v, 0], [0, 0, v/L], [0, 0, -1/tau_s]],   B = [0, 0, 1/tau_s]^T

Dead times: each actuator may also wait before it starts to move (vehicle.yaml
throttle_dead_time, brake_dead_time, steer_dead_time; 0 until measured). The actuator
tests fit them with the lags (analysis.fit_fopdt: a dead time, then a first-order lag),
and the model adds them to its loops' delays (speed_delay, lateral_delay), exactly in
the margins and as Pade for poles:

    P(s) e^(-s T_t),   T_t = throttle_dead_time

Units: SI and radians (REP-103: positive steering turns left). Every number in
vehicle.yaml's actuators section is a guess until the car-day tests (docs/actuator-tests.md).
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
import math
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy import signal

Poly = np.ndarray                      # polynomial coefficients, highest power first
TF = Tuple[Poly, Poly]                 # (numerator, denominator)


# --- The numbers --------------------------------------------------------------------------

@dataclass(frozen=True)
class SpeedLoopGains:
    """The spine's speed loop: spine-core speed_loop.rs SpeedLoopConfig::default."""

    kp: float = 0.3              # throttle per m/s of error
    ki: float = 0.1              # throttle per (m/s x s)
    integral_limit: float = 0.3  # the most throttle the integral may give (anti-windup)
    setpoint_weight: float = 1.0  # b: the share of the set speed the P term sees
    tick: float = 0.01           # s: SpineBoard::tick every 10 ms (esp32-umod main.rs TICK)
    accel_slew: float = 1.5      # m/s^2: the spine ramps its set speed (spine.rs max_accel)


@dataclass(frozen=True)
class SpeedLqiWeights:
    """The LQI's cost, J = sum(q_speed e^2 + q_integral z^2 + r_accel u^2) per tick (TODO 3)."""

    q_speed: float = 2.0      # per (m/s)^2 of speed error
    q_integral: float = 0.5   # per (m/s x s)^2 of its integral
    r_accel: float = 1.0      # per (m/s^2)^2 of the signed acceleration asked
    setpoint_weight: float = 0.5  # b: the share of the set speed the speed term sees


@dataclass(frozen=True)
class SpeedLqiGains:
    """
    The LQI's gains for spine-core (speed_loop.rs LqiConfig), from speed_lqi().

    u = d r + k_speed (b r - v) - k_accel (p - d r) + k_integral z,   z = sum((r - v) dt)
    u: the signed acceleration asked of the pedals (+ throttle, - brake), m/s^2;
    p: the pedals' acceleration, after their lag (the spine models it from what it sent).
    """

    k_speed: float      # m/s^2 per m/s
    k_accel: float      # per m/s^2
    k_integral: float   # m/s^2 per (m/s x s)
    setpoint_weight: float
    design_lag: float   # s: the actuator lag the gains were worked out for
    tick: float         # s


# spine-core's 'smooth' profile (SpeedLoopConfig::smooth, TODO 27): proposed for the car
# day, not the default. Real poles at -0.28 and -1.32 /s; b moves the zero to -0.33.
SMOOTH = SpeedLoopGains(kp=1.0, ki=0.25, integral_limit=1.0, setpoint_weight=0.75)


@dataclass(frozen=True)
class Delays:
    """
    The delays in each loop (s), and three cases: none, typical, worst.

    The lateral loop here is lane keeping by camera: a picture, its detections, the
    controller's next cycle, then the command on its way to the steering.
    """

    camera: float = 0.026           # picture to detections, median, Mac (camera-pipeline.md)
    camera_worst: float = 0.042     # ... the worst seen
    camera_period: float = 1 / 15   # 15 pictures a second
    control_period: float = 0.05    # MPPI and RPP at 20 Hz (nav2.yaml controller_frequency)
    heartbeat: float = 0.1          # the most a command may lag before the ESP32 brakes
    wheel_speed: float = 0.05       # GUESS: the wheel speed's age (time between pulses)

    def lateral(self, case: str) -> float:
        """Return the lateral loop's total delay for a case."""
        if case == 'none':
            return 0.0
        if case == 'typical':      # each sampler holds half a period on average
            return self.camera + self.camera_period / 2 + self.control_period / 2
        if case == 'worst':        # whole periods, and a command as stale as allowed
            return (self.camera_worst + self.camera_period + self.control_period
                    + self.heartbeat)
        raise ValueError(f'case must be none, typical or worst, not {case!r}')

    def speed(self, case: str, tick: float) -> float:
        """Return the spine's speed loop's delay: its tick, and the wheel speed's age."""
        if case == 'none':
            return 0.0
        if case == 'typical':
            return tick / 2 + self.wheel_speed / 2
        if case == 'worst':
            return tick + 2 * self.wheel_speed
        raise ValueError(f'case must be none, typical or worst, not {case!r}')


CASES = ('none', 'typical', 'worst')

# m/s: the speed today's ESP32 board takes full throttle to give, for its open speed loop
# (esp32-umod main.rs FULL_THROTTLE_SPEED; legacy.yaml full_throttle_speed). TODO(Q4).
FULL_THROTTLE_SPEED = 3.0


@dataclass(frozen=True)
class Lookahead:
    """Regulated Pure Pursuit's lookahead (nav2.yaml FollowPath): ld = clamp(v t, min, max)."""

    time: float = 1.5      # s: lookahead_time
    shortest: float = 1.5  # m: min_lookahead_dist
    longest: float = 4.0   # m: max_lookahead_dist

    def at(self, speed: float) -> float:
        """Return the lookahead distance (m) at a speed (m/s)."""
        return min(self.longest, max(self.shortest, abs(speed) * self.time))


# vehicle.yaml keys this model uses, and the ones it leaves out on purpose (nonlinear).
# Every key under actuators: is in one or the other (tools/tests/test_vehicle_model_tool.py
# checks, for each reader of that section).
YAML_KEYS = {
    'vehicle.wheelbase': 'wheelbase',
    'vehicle.max_steering_angle': 'max_steering',
    'actuators.max_accel': 'max_accel',
    'actuators.throttle_lag': 'throttle_lag',
    'actuators.throttle_dead_time': 'throttle_dead_time',
    'actuators.max_brake': 'max_brake',
    'actuators.brake_lag': 'brake_lag',
    'actuators.brake_dead_time': 'brake_dead_time',
    'actuators.drag': 'drag',
    'actuators.steer_rate': 'steer_rate',
    'actuators.steer_lag': 'steer_lag',
    'actuators.steer_dead_time': 'steer_dead_time',
}
# open_loop_decel: how the legacy bridge's open speed loop stops, for the geofence only.
NOT_LINEAR = {'actuators.throttle_deadband', 'actuators.steer_offset',
              'actuators.steer_deadband', 'actuators.open_loop_decel'}


@dataclass(frozen=True)
class VehicleModel:
    """The car's numbers (vehicle.yaml), the spine's gains, and the loops' delays."""

    wheelbase: float = 1.57          # m
    max_steering: float = 0.50       # rad
    max_accel: float = 1.5           # m/s^2 at full throttle. GUESS: test 4.1, 4.4
    throttle_lag: float = 0.1        # s. GUESS: test 4.1
    throttle_dead_time: float = 0.0  # s before the throttle moves. Not measured: test 4.1
    max_brake: float = 3.0           # m/s^2 at full brake. GUESS: test 4.3
    brake_lag: float = 0.2           # s. GUESS: test 4.3
    brake_dead_time: float = 0.0     # s before the brake moves. Not measured: test 4.3
    drag: float = 0.1                # 1/s. GUESS: test 4.2
    steer_rate: float = 0.8          # rad/s, the Jrk motor's top speed. GUESS: test 3.11
    steer_lag: float = 0.1           # s, the linkage. GUESS: test 3.11, 4.5
    steer_dead_time: float = 0.0     # s before the motor moves. Not measured: test 3.11
    steer_step: float = 0.1          # rad: the step size the rate limit is linearised for
    steer_zeta: Optional[float] = None   # damping, if measured (test 3.11's overshoot)
    gains: SpeedLoopGains = SpeedLoopGains()
    delays: Delays = Delays()
    lookahead: Lookahead = Lookahead()

    # --- speed ----------------------------------------------------------------------------

    @property
    def speed_gain(self) -> float:
        """K (m/s per unit throttle): the speed full throttle would hold, a_max / d."""
        return self.max_accel / self.drag

    @property
    def speed_tau(self) -> float:
        """tau_v (s): the car's own time constant, 1 / d."""
        return 1.0 / self.drag

    def car_tf(self) -> TF:
        """Return the car alone, first order: V/U_a = K / (tau_v s + 1)."""
        return np.array([self.speed_gain]), np.array([self.speed_tau, 1.0])

    def throttle_tf(self) -> TF:
        """Return the throttle actuator: U_a/U = 1 / (tau_t s + 1)."""
        return np.array([1.0]), np.array([self.throttle_lag, 1.0])

    def speed_plant_tf(self, pade_order: int = 0) -> TF:
        """
        Throttle command to speed: the throttle's lag, then the car.

        With pade_order > 0, times e^(-s throttle_dead_time) as a Pade approximation (for
        poles and simulation; margins() takes the delay exactly: speed_delay).
        """
        tf = series(self.throttle_tf(), self.car_tf())
        return with_delay(tf, self.throttle_dead_time, pade_order) if pade_order else tf

    def brake_plant_tf(self, pade_order: int = 0) -> TF:
        """
        Brake command to speed while moving forward: -b_max / ((s + d)(tau_b s + 1)).

        With pade_order > 0, times e^(-s brake_dead_time) as a Pade approximation.
        """
        tf = (np.array([-self.max_brake]),
              np.polymul([1.0, self.drag], [self.brake_lag, 1.0]))
        return with_delay(tf, self.brake_dead_time, pade_order) if pade_order else tf

    def speed_delay(self, case: str) -> float:
        """Return the speed loop's whole delay (s): the loop's, and the throttle's dead time."""
        return self.delays.speed(case, self.gains.tick) + self.throttle_dead_time

    def lateral_delay(self, case: str) -> float:
        """Return the lateral loop's whole delay (s): the loop's, and the steering's dead time."""
        return self.delays.lateral(case) + self.steer_dead_time

    def speed_controller_tf(self, gains: Optional[SpeedLoopGains] = None) -> TF:
        """Return the spine's PI: C(s) = kp + ki / s = (kp s + ki) / s."""
        g = gains or self.gains
        if g.ki == 0.0:                  # P only: no pole at the origin
            return np.array([g.kp]), np.array([1.0])
        return np.array([g.kp, g.ki]), np.array([1.0, 0.0])

    def speed_open_loop(self, gains: Optional[SpeedLoopGains] = None,
                        throttle_lag: bool = True) -> TF:
        """L(s) = C(s) P(s): the speed loop broken at the measurement."""
        plant = self.speed_plant_tf() if throttle_lag else self.car_tf()
        return series(self.speed_controller_tf(gains), plant)

    def speed_reference_tf(self, gains: Optional[SpeedLoopGains] = None) -> TF:
        """Return the PI's path from the set speed: C_r(s) = b kp + ki / s."""
        g = gains or self.gains
        if g.ki == 0.0:
            return np.array([g.setpoint_weight * g.kp]), np.array([1.0])
        return np.array([g.setpoint_weight * g.kp, g.ki]), np.array([1.0, 0.0])

    def speed_closed_loop(self, gains: Optional[SpeedLoopGains] = None,
                          throttle_lag: bool = True) -> TF:
        """
        Set speed to speed: T(s) = C_r P / (1 + C P); with b = 1, L / (1 + L).

        C and C_r share their denominator (s, or 1 for P only), so the poles are 1 + L's
        and only the zero moves with b: to -ki / (b kp).
        """
        g = gains or self.gains
        if g.setpoint_weight == 1.0:
            return feedback(self.speed_open_loop(g, throttle_lag))
        pn, pd = self.speed_plant_tf() if throttle_lag else self.car_tf()
        cn, cd = self.speed_controller_tf(g)
        rn, _ = self.speed_reference_tf(g)
        return (trim(np.polymul(rn, pn)),
                trim(np.polyadd(np.polymul(cd, pd), np.polymul(cn, pn))))

    def place_speed_gains(self, zeta: float, wn: float) -> SpeedLoopGains:
        """
        Return the PI gains that put the speed loop's poles at w_n and zeta (pole placement).

        Without the throttle's lag the closed loop is s^2 + (d + a kp) s + a ki; matching
        s^2 + 2 zeta w_n s + w_n^2 gives kp = (2 zeta w_n - d) / a, ki = w_n^2 / a.
        Check the result with the lag and the delays (speed_closed_loop, margins).
        """
        a, d = self.max_accel, self.drag
        return replace(self.gains, kp=(2.0 * zeta * wn - d) / a, ki=wn * wn / a)

    def speed_lqi(self, weights: SpeedLqiWeights = SpeedLqiWeights(),
                  tick: Optional[float] = None) -> SpeedLqiGains:
        """
        Return the LQI's gains for the speed and brake: one signed acceleration (TODO 3).

        The pedals are inverted through the model (u > 0: throttle u / a_max; u < 0: brake
        -u / b_max), so the plant from u is the same both ways but for the lag. State, in
        error coordinates about a set speed r (p~ = p - d r, the pedals beyond the drag):

            e = r - v,  e' = -d e - p~;   p~' = (u~ - p~) / tau;   z' = e
            A = [[-d, -1, 0], [0, -1/tau, 0], [1, 0, 0]],   B = [0, 1/tau, 0]^T

        tau is the slower of the throttle's and the brake's lags (one set of gains for
        both). Held for a tick (ZOH), then the discrete Riccati equation: u~ = -K x.
        Worked out here, once: spine-core keeps the result as constants (no matrices on
        the ESP32). `pixi run vehicle-model --lqi` prints them.
        """
        from scipy import linalg
        dt = tick or self.gains.tick
        tau = max(self.throttle_lag, self.brake_lag)
        d = self.drag
        a = np.array([[-d, -1.0, 0.0], [0.0, -1.0 / tau, 0.0], [1.0, 0.0, 0.0]])
        b = np.array([[0.0], [1.0 / tau], [0.0]])
        phi, gam, _, _, _ = signal.cont2discrete((a, b, np.eye(3), np.zeros((3, 1))), dt,
                                                 method='zoh')
        q = np.diag([weights.q_speed, 0.0, weights.q_integral])
        r = np.array([[weights.r_accel]])
        p = linalg.solve_discrete_are(phi, gam, q, r)
        k = np.linalg.solve(r + gam.T @ p @ gam, gam.T @ p @ phi)[0]
        # u~ = -(k0 e + k1 p~ + k2 z); k0 and k2 come out negative (more speed error, more push).
        return SpeedLqiGains(k_speed=-float(k[0]), k_accel=float(k[1]),
                             k_integral=-float(k[2]),
                             setpoint_weight=weights.setpoint_weight,
                             design_lag=tau, tick=dt)

    def speed_lqi_open_loop(self, g: SpeedLqiGains, model_accel: Optional[float] = None,
                            model_lag: Optional[float] = None) -> TF:
        """
        Return the LQI's loop broken at the speed measurement, on the throttle side.

        The spine models the pedals from what it sent, p^ = u / (tau_m s + 1), so k_accel
        feeds back the loop's own command (no plant in that path):

            u = (k_speed + k_integral / s)(-v) - k_accel p^      (the reference left out)
            L(s) = (k_speed + k_integral / s) P_v(s) (tau_m s + 1) / (tau_m s + 1 + k_accel)
            P_v(s) = (a_max / a_model) / ((tau_t s + 1)(s + d))

        model_accel and model_lag are the a_max and lag the spine assumes (the gains'
        model: this model's own, unless the car differs from it).
        """
        gain = self.max_accel / (model_accel or self.max_accel)
        tau_m = model_lag if model_lag is not None else self.throttle_lag
        plant = (np.array([gain]), np.polymul([self.throttle_lag, 1.0], [1.0, self.drag]))
        pi = (np.array([g.k_speed, g.k_integral]), np.array([1.0, 0.0]))
        inner = (np.array([tau_m, 1.0]), np.array([tau_m, 1.0 + g.k_accel]))
        return series(pi, plant, inner)

    def speed_state_space(self) -> Tuple[np.ndarray, ...]:
        """Return the plant: x' = A x + B u, y = C x; x = [v, u_a], u = throttle, y = v."""
        a = np.array([[-self.drag, self.max_accel],
                      [0.0, -1.0 / self.throttle_lag]])
        b = np.array([[0.0], [1.0 / self.throttle_lag]])
        c = np.array([[1.0, 0.0]])
        return a, b, c, np.zeros((1, 1))

    def open_loop_speed(self, target: float, full_throttle_speed: float) -> float:
        """
        Return the speed today's open loop settles at (no wheel speed on the ESP32 board).

        It sets throttle = target / full_throttle_speed; the final value theorem on
        K / (tau_v s + 1) gives v = K x throttle.
        """
        return self.speed_gain * target / full_throttle_speed

    # --- steering -------------------------------------------------------------------------

    @property
    def motor_tau(self) -> float:
        """tau_m (s): the lag whose mean delay matches the rate limit, for a steer_step."""
        return self.steer_step / (2.0 * self.steer_rate)

    def steer_wn_zeta(self) -> Tuple[float, float]:
        """Return (w_n rad/s, zeta) of the steering's second-order model."""
        tm, tl = self.motor_tau, self.steer_lag
        if tl <= 0.0:
            raise ValueError('steer_lag must be positive for a second-order steering model')
        wn = 1.0 / math.sqrt(tm * tl)
        zeta = (tm + tl) / (2.0 * math.sqrt(tm * tl)) if self.steer_zeta is None \
            else self.steer_zeta
        return wn, zeta

    def steer_tf(self) -> TF:
        """Steering command to wheel angle: w_n^2 / (s^2 + 2 zeta w_n s + w_n^2)."""
        return second_order(*self.steer_wn_zeta())

    @property
    def steer_tau(self) -> float:
        """tau_s (s): one lag with the same mean delay as the steering, 2 zeta / w_n."""
        wn, zeta = self.steer_wn_zeta()
        return 2.0 * zeta / wn

    # --- lateral --------------------------------------------------------------------------

    def lateral(self, speed: float, steering_order: int = 1) -> Tuple[np.ndarray, ...]:
        """
        Return (A, B, C, D) of the path errors at a speed (m/s), about driving straight.

        steering_order 1: x = [e_y, e_theta, delta], the steering as one lag tau_s.
        steering_order 2: x = [e_y, e_theta, delta, delta'], the second-order steering.
        y = [e_y, e_theta].
        """
        v, L = speed, self.wheelbase
        if steering_order == 1:
            ts = self.steer_tau
            a = np.array([[0.0, v, 0.0],
                          [0.0, 0.0, v / L],
                          [0.0, 0.0, -1.0 / ts]])
            b = np.array([[0.0], [0.0], [1.0 / ts]])
        elif steering_order == 2:
            wn, zeta = self.steer_wn_zeta()
            a = np.array([[0.0, v, 0.0, 0.0],
                          [0.0, 0.0, v / L, 0.0],
                          [0.0, 0.0, 0.0, 1.0],
                          [0.0, 0.0, -wn * wn, -2.0 * zeta * wn]])
            b = np.array([[0.0], [0.0], [0.0], [wn * wn]])
        else:
            raise ValueError(f'steering_order must be 1 or 2, not {steering_order}')
        c = np.zeros((2, a.shape[0]))
        c[0, 0] = c[1, 1] = 1.0
        return a, b, c, np.zeros((2, 1))

    def pursuit_gains(self, speed: float, lookahead: Optional[float] = None) -> np.ndarray:
        """
        Return K for delta_cmd = -K x: pure pursuit, linearised.

        Pure pursuit steers for the curvature 2 sin(alpha) / ld to a point ld ahead on the
        path; for small errors alpha = -(e_theta + e_y / ld), so
        delta = -(2 L / ld^2) e_y - (2 L / ld) e_theta.
        """
        ld = self.lookahead.at(speed) if lookahead is None else lookahead
        L = self.wheelbase
        return np.array([[2.0 * L / ld ** 2, 2.0 * L / ld, 0.0]])

    def lateral_closed_loop(self, speed: float, lookahead: Optional[float] = None) -> np.ndarray:
        """Return A - B K: the path errors with pure pursuit steering (3 states)."""
        a, b, _, _ = self.lateral(speed)
        return a - b @ self.pursuit_gains(speed, lookahead)

    def lateral_open_loop(self, speed: float, lookahead: Optional[float] = None) -> TF:
        """L(s) = K (sI - A)^-1 B: the lateral loop broken at the steering command."""
        a, b, _, _ = self.lateral(speed)
        k = self.pursuit_gains(speed, lookahead)
        num, den = signal.ss2tf(a, b, k, np.zeros((1, 1)))
        return trim(num[0]), den

    # --- discrete time --------------------------------------------------------------------

    def speed_loop_discrete(self, gains: Optional[SpeedLoopGains] = None) -> np.ndarray:
        """
        Return the closed speed loop's matrix as the spine runs it, every tick.

        The plant held for a tick (zero-order hold), then the spine's own update:
        I[k+1] = I[k] + ki dt e[k];  u[k] = kp e[k] + I[k+1].  State [v, u_a, I].
        """
        g = gains or self.gains
        a, b, c, d = self.speed_state_space()
        phi, gam, _, _, _ = signal.cont2discrete((a, b, c, d), g.tick, method='zoh')
        kp = g.kp + g.ki * g.tick          # e[k] reaches u[k] through both terms
        top = np.hstack([phi - kp * gam @ c, gam])
        bottom = np.hstack([-g.ki * g.tick * c, np.ones((1, 1))])
        return np.vstack([top, bottom])

    def lateral_discrete(self, speed: float, period: Optional[float] = None,
                         lookahead: Optional[float] = None, delay_steps: int = 1) -> np.ndarray:
        """
        Return the closed lateral loop's matrix at the controller's rate.

        The plant held for a period (ZOH); the command computed from x[k] goes out
        delay_steps periods later (1: it's sent at the end of the cycle that read x[k]).
        """
        T = period or self.delays.control_period
        a, b, c, d = self.lateral(speed)
        phi, gam, _, _, _ = signal.cont2discrete((a, b, c, d), T, method='zoh')
        k = self.pursuit_gains(speed, lookahead)
        n = phi.shape[0]
        if delay_steps == 0:
            return phi - gam @ k
        m = delay_steps
        # State [x, u[k-m], ..., u[k-1]]: a shift register of commands on their way.
        big = np.zeros((n + m, n + m))
        big[:n, :n] = phi
        big[:n, n:n + 1] = gam
        for i in range(m - 1):
            big[n + i, n + i + 1] = 1.0
        big[n + m - 1, :n] = -k[0]
        return big

    # --- refit from the car ---------------------------------------------------------------

    def with_measured(self, measured: Optional[Mapping] = None, **values) -> 'VehicleModel':
        """
        Return a copy with measured numbers in place of the guesses.

        `measured` is the actuator tests' report (Report.yaml: 'actuators.steer_lag' ->
        (value, comment)), or any mapping of those keys or the field names to numbers.
        `steer_overshoot` (0..1, read off test 3.11's step) sets the steering's damping.
        Keys the linear model leaves out (deadbands, the centre offset) are skipped.
        """
        merged: Dict[str, object] = dict(measured or {})
        merged.update(values)
        names = {f.name for f in fields(self)}
        changes: Dict[str, float] = {}
        for key, value in merged.items():
            if isinstance(value, (tuple, list)):
                value = value[0]
            if key in NOT_LINEAR:
                continue
            if key == 'steer_overshoot':
                changes['steer_zeta'] = zeta_from_overshoot(float(value))
                continue
            name = YAML_KEYS.get(key, key)
            if name not in names or name in ('gains', 'delays', 'lookahead'):
                raise KeyError(f'not a number this model uses: {key!r}')
            changes[name] = float(value)
        return replace(self, **changes)


def from_vehicle_yaml(path: str, **overrides) -> VehicleModel:
    """Build the model from vehicle.yaml (its vehicle: and actuators: sections)."""
    import yaml
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return from_config(cfg, **overrides)


def from_config(cfg: Mapping, **overrides) -> VehicleModel:
    """Build the model from vehicle.yaml already loaded (aav_description.vehicle.load())."""
    flat = {}
    for key in YAML_KEYS:
        section, name = key.split('.')
        value = (cfg.get(section) or {}).get(name)
        if value is not None:
            flat[key] = float(value)
    return VehicleModel().with_measured(flat, **overrides)


# --- Transfer functions -------------------------------------------------------------------

def trim(p: Sequence[float]) -> Poly:
    """Drop leading (near-)zero coefficients."""
    p = np.atleast_1d(np.asarray(p, dtype=float))
    scale = max(np.max(np.abs(p)), 1e-300)
    nz = np.nonzero(np.abs(p) > 1e-12 * scale)[0]
    return p[nz[0]:] if len(nz) else np.array([0.0])


def series(*tfs: TF) -> TF:
    """G1 G2 ...: blocks one after another."""
    num, den = np.array([1.0]), np.array([1.0])
    for n, d in tfs:
        num, den = np.polymul(num, n), np.polymul(den, d)
    return trim(num), trim(den)


def feedback(open_loop: TF) -> TF:
    """L / (1 + L): unity negative feedback."""
    n, d = open_loop
    return trim(n), trim(np.polyadd(d, n))


def second_order(wn: float, zeta: float) -> TF:
    """w_n^2 / (s^2 + 2 zeta w_n s + w_n^2)."""
    return np.array([wn * wn]), np.array([1.0, 2.0 * zeta * wn, wn * wn])


def pade(delay: float, order: int = 2) -> TF:
    """Return the Pade approximation of e^(-s delay) as (num, den)."""
    if delay <= 0.0:
        return np.array([1.0]), np.array([1.0])
    c = [math.factorial(2 * order - k) * math.factorial(order)
         / (math.factorial(2 * order) * math.factorial(k) * math.factorial(order - k))
         for k in range(order + 1)]
    den = np.array([c[k] * delay ** k for k in range(order, -1, -1)])
    num = np.array([c[k] * (-delay) ** k for k in range(order, -1, -1)])
    return num / den[0], den / den[0]


def poles(tf: TF) -> np.ndarray:
    """Return the denominator's roots, slowest first."""
    p = np.roots(tf[1])
    return p[np.argsort(-p.real)]


def dc_gain(tf: TF) -> float:
    """G(0), the final value of a unit step's response (if G is stable)."""
    n, d = tf
    return float(np.polyval(n, 0.0) / np.polyval(d, 0.0))


def cancel_origin(tf: TF) -> TF:
    """Cancel the factors of s that the numerator and the denominator share."""
    n, d = trim(tf[0]), trim(tf[1])
    while len(n) > 1 and len(d) > 1 and abs(n[-1]) < 1e-12 and abs(d[-1]) < 1e-12:
        n, d = n[:-1], d[:-1]
    return n, d


def final_value_step_error(open_loop: TF) -> float:
    """
    Return the steady-state error to a unit step, by the final value theorem.

    e_ss = lim_{s->0} s (1 / (1 + L(s))) (1 / s) = 1 / (1 + L(0)); 0 when L has a pole at 0.
    """
    n, d = cancel_origin(open_loop)
    if abs(np.polyval(d, 0.0)) < 1e-12:
        return 0.0
    return 1.0 / (1.0 + np.polyval(n, 0.0) / np.polyval(d, 0.0))


def velocity_constant(open_loop: TF) -> float:
    """K_v = lim_{s->0} s L(s): a ramp of slope r leaves the error r / K_v."""
    n, d = cancel_origin(open_loop)
    d0 = np.polyval(d, 0.0)
    if abs(d0) > 1e-12:
        return 0.0
    # d(s) = s d1(s): K_v = n(0) / d1(0).
    d1 = np.polydiv(d, [1.0, 0.0])[0]
    if abs(np.polyval(d1, 0.0)) < 1e-12:
        return math.inf
    return float(np.polyval(n, 0.0) / np.polyval(d1, 0.0))


# --- Step responses -----------------------------------------------------------------------

@dataclass(frozen=True)
class StepInfo:
    """A step response's numbers. Times in s; overshoot as a fraction of the final value."""

    final: float
    rise_time: float       # 10% to 90%
    peak_time: float
    overshoot: float
    settling_time: float   # last time outside 2% of the final value


def step(tf: TF, t_end: Optional[float] = None, n: int = 20001) -> Tuple[np.ndarray, np.ndarray]:
    """Return (t, y): the unit step response."""
    if t_end is None:
        slowest = min((-p.real for p in poles(tf) if p.real < -1e-9), default=1.0)
        t_end = 10.0 / slowest
    t = np.linspace(0.0, t_end, n)
    t, y = signal.step(signal.lti(*tf), T=t)
    return t, y


def step_info(t: np.ndarray, y: np.ndarray, final: Optional[float] = None,
              band: float = 0.02) -> StepInfo:
    """Read the rise time, peak, overshoot and settling time off a sampled step."""
    yf = float(y[-1]) if final is None else final

    def crossing(level):
        idx = np.nonzero(y >= level)[0] if yf > 0 else np.nonzero(y <= level)[0]
        if not len(idx):
            return math.nan
        i = idx[0]
        if i == 0:
            return float(t[0])
        return float(t[i - 1] + (t[i] - t[i - 1]) * (level - y[i - 1]) / (y[i] - y[i - 1]))

    rise = crossing(0.9 * yf) - crossing(0.1 * yf)
    ipk = int(np.argmax(y) if yf > 0 else np.argmin(y))
    overshoot = max(0.0, (y[ipk] - yf) / yf)
    outside = np.nonzero(np.abs(y - yf) > band * abs(yf))[0]
    settle = float(t[outside[-1] + 1]) if len(outside) and outside[-1] + 1 < len(t) else (
        math.nan if len(outside) else 0.0)
    return StepInfo(yf, rise, float(t[ipk]), overshoot, settle)


def second_order_overshoot(zeta: float) -> float:
    """Return the textbook overshoot, exp(-zeta pi / sqrt(1 - zeta^2)); 0 if zeta >= 1."""
    if zeta >= 1.0:
        return 0.0
    return math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta * zeta))


def zeta_from_overshoot(overshoot: float) -> float:
    """Return the damping that gives a measured overshoot (0..1): the formula, inverted."""
    if overshoot <= 0.0:
        return 1.0
    ln = math.log(overshoot)
    return -ln / math.sqrt(math.pi ** 2 + ln * ln)


def wn_zeta(pole: complex) -> Tuple[float, float]:
    """Return a pole's natural frequency (rad/s) and damping."""
    wn = abs(pole)
    return wn, (-pole.real / wn if wn > 0 else 1.0)


# --- Frequency response: Bode and margins -------------------------------------------------

def freq_response(tf: TF, w: np.ndarray, delay: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Return (magnitude, phase in degrees) of G(jw) e^(-jw delay).

    The phase is summed pole by pole and zero by zero, so it is continuous (unwrapped)
    and starts at -90 deg per pole at the origin.
    """
    num, den = tf
    jw = 1j * w
    mag = np.abs(np.polyval(num, jw) / np.polyval(den, jw))
    phase = np.zeros_like(w)
    for z in np.roots(num):
        phase += np.unwrap(np.angle(jw - z))
    for p in np.roots(den):
        phase -= np.unwrap(np.angle(jw - p))
    if num[0] / den[0] < 0:
        phase -= math.pi
    phase -= w * delay
    return mag, np.degrees(phase)


@dataclass(frozen=True)
class Margins:
    """Gain and phase margins, and the closed loop's bandwidth."""

    gain_margin_db: float      # inf when the phase never reaches -180 after crossover
    phase_margin_deg: float
    gain_crossover: float      # rad/s, |L| = 1
    phase_crossover: float     # rad/s, phase = -180 (nan: none)
    bandwidth: float           # rad/s, |T| 3 dB under its low-frequency value
    max_delay: float           # s: the extra delay that would use up the phase margin


def margins(open_loop: TF, delay: float = 0.0,
            w: Optional[np.ndarray] = None) -> Margins:
    """Return the margins of L(s) e^(-s delay), with the delay exact (not Pade)."""
    if w is None:
        w = np.logspace(-4, 4, 40001)
    mag, ph = freq_response(open_loop, w, delay)
    db = 20 * np.log10(mag)

    def at_level(i, y, level):
        """Return the w where y crosses level between samples i and i + 1 (log w)."""
        frac = (level - y[i]) / (y[i + 1] - y[i])
        return float(np.exp(np.log(w[i]) + frac * (np.log(w[i + 1]) - np.log(w[i]))))

    down = np.nonzero((db[:-1] >= 0) & (db[1:] < 0))[0]
    if len(down):
        i = down[-1]
        wc = at_level(i, db, 0.0)
        pm = 180.0 + float(np.interp(np.log(wc), np.log(w[i:i + 2]), ph[i:i + 2]))
        # Keep the phase in the turn where the loop is: -540..-180 counts as -180 short.
        pm = (pm + 180.0) % 360.0 - 180.0
    else:
        wc, pm = math.nan, math.inf
    # The phase crossover that matters: the first -180 (mod 360) crossing above wc.
    shifted = (ph + 180.0) / 360.0
    turns = np.floor(shifted)
    cross = np.nonzero((turns[:-1] != turns[1:]) & (w[:-1] >= (wc if wc == wc else 0)))[0]
    if len(cross):
        i = cross[0]
        w180 = at_level(i, shifted, max(turns[i], turns[i + 1]))
        m180, _ = freq_response(open_loop, np.array([w180]), delay)
        gm = -20 * math.log10(m180[0])
    else:
        w180, gm = math.nan, math.inf
    # Closed loop: T = L / (1 + L), delay included.
    lj = mag * np.exp(1j * np.radians(ph))
    t = np.abs(lj / (1 + lj))
    ref = t[0]
    under = np.nonzero(t < ref / math.sqrt(2.0))[0]
    bw = float(w[under[0]]) if len(under) else math.inf
    max_delay = math.radians(pm) / wc if wc == wc and pm > 0 else 0.0
    return Margins(gm, pm, wc, w180, bw, max_delay)


def with_delay(tf: TF, delay: float, order: int = 2) -> TF:
    """G(s) with a Pade-approximated delay in series."""
    return series(tf, pade(delay, order))


# --- The summary for pixi run vehicle-model -----------------------------------------------

def lateral_speeds() -> Tuple[float, ...]:
    """Return the speeds the tables use: ego's 1 m/s cap now, the spine's 3 later."""
    return (0.5, 1.0, 2.0, 3.0)


def summary(model: VehicleModel) -> List[Tuple[str, List[List[str]]]]:
    """Return the tables: (title, rows), the first row a header."""
    g = model.gains
    out = []

    # Speed.
    rows = [['loop', 'poles (1/s)', 'w_n, zeta', 'rise s', 'overshoot', 'settle s',
             'e_ss step']]
    for label, tf, ol in (
            ('car alone (first order)', model.car_tf(), None),
            ('car + throttle lag', model.speed_plant_tf(), None),
            ('spine PI, no lag', model.speed_closed_loop(throttle_lag=False),
             model.speed_open_loop(throttle_lag=False)),
            ('spine PI + throttle lag', model.speed_closed_loop(),
             model.speed_open_loop()),
            ("'smooth' profile + throttle lag", model.speed_closed_loop(SMOOTH),
             model.speed_open_loop(SMOOTH))):
        t, y = step(tf)
        info = step_info(t, y)
        p = poles(tf)
        cpx = [q for q in p if q.imag > 1e-9]
        wz = '%.3f, %.2f' % wn_zeta(cpx[0]) if cpx else '-'
        ess = '-' if ol is None else f'{final_value_step_error(ol):.3f}'
        rows.append([label, fmt_poles(p), wz, f'{info.rise_time:.2f}',
                     f'{100 * info.overshoot:.1f}%', f'{info.settling_time:.1f}', ess])
    out.append(("Speed: throttle to speed, and the spine's PI around it", rows))

    rows = [['delay case', 'delay s', 'phase margin', 'gain margin', 'crossover rad/s',
             'bandwidth rad/s']]
    for case in CASES:
        T = model.speed_delay(case)
        m = margins(model.speed_open_loop(), T)
        rows.append([case, f'{T:.3f}', f'{m.phase_margin_deg:.1f} deg', fmt_db(m.gain_margin_db),
                     f'{m.gain_crossover:.3f}', f'{m.bandwidth:.3f}'])
    out.append(('Speed loop margins (spine PI + throttle lag)', rows))

    p_only = replace(g, ki=0.0)
    kv = velocity_constant(model.speed_open_loop())
    rows = [['case', 'by the final value theorem', 'value'],
            ["step, the spine's PI", 'e = 1 / (1 + L(0)), L(0) = inf', '0 m/s'],
            ['step, P only (ki = 0)', f'e = 1 / (1 + kp K) = 1 / (1 + {g.kp:g} x '
             f'{model.speed_gain:g})',
             f'{final_value_step_error(model.speed_open_loop(p_only)):.3f} of the step'],
            [f"ramp at the spine's slew, {g.accel_slew:g} m/s^2",
             f'e = r / K_v, K_v = ki K = {kv:g} 1/s', f'{g.accel_slew / kv:.2f} m/s behind'],
            ["open loop (today's ESP32: no wheel speed)",
             f'v = K x target / {FULL_THROTTLE_SPEED:g}',
             f'{model.open_loop_speed(1.0, FULL_THROTTLE_SPEED):g} x the target']]
    out.append(('Steady state (speed)', rows))

    # Steering.
    wn, zeta = model.steer_wn_zeta()
    t, y = step(model.steer_tf())
    info = step_info(t, y)
    rows = [['tau_m s', 'tau_l s', 'w_n rad/s', 'zeta', 'tau_s s', 'rise s', 'overshoot',
             'settle s'],
            [f'{model.motor_tau:.4f}', f'{model.steer_lag:.3f}', f'{wn:.2f}', f'{zeta:.3f}',
             f'{model.steer_tau:.4f}', f'{info.rise_time:.3f}', f'{100 * info.overshoot:.1f}%',
             f'{info.settling_time:.3f}']]
    out.append((f'Steering: second order, for a {model.steer_step:g} rad step '
                f'(rate limit {model.steer_rate:g} rad/s)', rows))

    # Lateral.
    rows = [['v m/s', 'ld m', 'open-loop eig', 'closed-loop poles (pure pursuit)',
             'w_n, zeta']]
    for v in lateral_speeds():
        a, _, _, _ = model.lateral(v)
        cl = np.linalg.eigvals(model.lateral_closed_loop(v))
        cl = cl[np.argsort(-cl.real)]
        cpx = [q for q in cl if q.imag > 1e-9]
        wz = '%.2f, %.2f' % wn_zeta(cpx[0]) if cpx else '-'
        rows.append([f'{v:g}', f'{model.lookahead.at(v):.2f}',
                     fmt_poles(np.linalg.eigvals(a)), fmt_poles(cl), wz])
    out.append(('Lateral: path errors, linearised about driving straight', rows))

    rows = [['v m/s'] + [f'PM / GM, {c} ({model.lateral_delay(c) * 1000:.0f} ms)'
                         for c in CASES] + ['crossover', 'delay left']]
    for v in lateral_speeds():
        ol = model.lateral_open_loop(v)
        ms = [margins(ol, model.lateral_delay(c)) for c in CASES]
        rows.append([f'{v:g}'] + [f'{m.phase_margin_deg:.0f} deg / {fmt_db(m.gain_margin_db)}'
                                  for m in ms]
                    + [f'{ms[0].gain_crossover:.2f} rad/s',
                       f'{1000 * ms[1].max_delay:.0f} ms'])
    out.append(('Lateral margins (pure pursuit + steering), by delay', rows))

    # Discrete.
    rows = [['loop', 'period s', 'largest |z|', 'continuous e^(s T)']]
    zs = np.linalg.eigvals(model.speed_loop_discrete())
    sc = poles(model.speed_closed_loop())
    rows.append(['speed (spine)', f'{g.tick:g}', f'{max(abs(zs)):.5f}',
                 f'{max(abs(np.exp(sc * g.tick))):.5f}'])
    for v in (1.0, 3.0):
        T = model.delays.control_period
        zl = np.linalg.eigvals(model.lateral_discrete(v))
        cl = np.linalg.eigvals(model.lateral_closed_loop(v))
        rows.append([f'lateral at {v:g} m/s, 1 period late', f'{T:g}', f'{max(abs(zl)):.4f}',
                     f'{max(abs(np.exp(cl * T))):.4f}'])
    out.append(('Discrete time, as the computers run it (|z| < 1: stable)', rows))
    return out


def fmt_poles(p: Sequence[complex]) -> str:
    """Poles as short text: -0.27±0.27j, -10."""
    parts, seen = [], set()
    for q in p:
        re, im = (0.0 if abs(q.real) < 1e-9 else q.real), abs(q.imag)
        if q.imag < -1e-9:
            continue
        key = (round(re, 4), round(im, 4))
        if im > 1e-9:
            parts.append(f'{re:.3g}±{im:.3g}j')
        elif key in seen:
            parts.append(f'{re:.3g} (twice)')
            parts.remove(f'{re:.3g}')
        else:
            parts.append(f'{re:.3g}')
        seen.add(key)
    return ', '.join(parts)


def fmt_db(db: float) -> str:
    return 'inf' if math.isinf(db) else f'{db:.1f} dB'
