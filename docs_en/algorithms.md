# Path-tracking algorithms in detail

This document derives the control law of every algorithm in the repository,
analyses its closed-loop behaviour, maps each symbol onto the corresponding
configuration field in the code, and compares the theory with the numbers
measured on the benchmark route.

- [0. Common framework](#0-common-framework)
- [1. Pure Pursuit](#1-pure-pursuit)
- [2. Stanley](#2-stanley)
- [3. MPC (LTV / Riccati)](#3-mpc-ltv--riccati)
- [4. MPPI](#4-mppi)
- [5. ACC longitudinal law](#5-acc-longitudinal-law)
- [6. Side-by-side summary](#6-side-by-side-summary)
- [7. References](#7-references)

Companion documents: [`comparison_report.md`](comparison_report.md) (results and
figures), [`../README.md`](../README.md) (build and run).

---

## 0. Common framework

All four controllers see the same plant, the same path representation and the
same longitudinal law, so any difference in the results comes from the steering
law alone.

### 0.1 Vehicle model

A kinematic bicycle model referenced to the **rear axle**
([`vehicle.hpp`](../include/path_tracking/vehicle.hpp)):

$$
\dot{x} = v\cos\psi,\qquad
\dot{y} = v\sin\psi,\qquad
\dot{\psi} = \frac{v}{L}\tan\delta,\qquad
\dot{v} = a
$$

with wheelbase $L = 2.7\,\mathrm{m}$, heading $\psi$, front-wheel steering
angle $\delta$ (positive = left) and longitudinal acceleration $a$. The
simulator integrates with explicit Euler at $\Delta t = 0.01\,\mathrm{s}$
(100 Hz), updating $v$, then the position, then $\psi$:

$$
v_{k+1} = \max(0,\; v_k + a_k\Delta t),\quad
x_{k+1} = x_k + v_{k+1}\cos\psi_k\,\Delta t,\quad
\psi_{k+1} = \psi_k + \frac{v_{k+1}\tan\delta_k}{L}\Delta t
$$

There is no mass, tyre or actuator-lag model: at these speeds
($\le 8\,\mathrm{m/s}$, $\le 1.5\,\mathrm{m/s^2}$ lateral) the kinematic model
is a fair approximation, and it keeps the comparison about the *algorithms*.

The steady-state steering angle that holds a path curvature $\kappa$ follows
directly from $\dot\psi = v\kappa$:

$$
\boxed{\;\delta_{\mathrm{ss}} = \arctan(L\kappa)\;}
$$

This identity is the curvature feed-forward used by MPC (§3.5) and the reason
Pure Pursuit is exact on circular arcs (§1.4).

### 0.2 Path and error definitions

The path is a polyline of samples $p_i = (x_i, y_i, \psi_i, \kappa_i, s_i, v_{\mathrm{ref},i})$
spaced $\Delta s \approx 0.54\,\mathrm{m}$ apart
([`path_io.hpp`](../include/path_tracking/path_io.hpp)). For a query point
$(x, y)$ the controllers take the nearest sample index

$$
i^\* = \arg\min_{i \in [\,\hat\imath - W,\; \hat\imath + W\,]}\; (x_i - x)^2 + (y_i - y)^2
$$

over a window $W$ around the previous index $\hat\imath$ (a warm start: $W = 300$
for the controllers, $200$ inside MPPI rollouts). Signed **cross-track error**
is the projection onto the left normal $\hat n = (-\sin\psi_{i^\*}, \cos\psi_{i^\*})$:

$$
e = -\sin\psi_{i^\*}\,(x - x_{i^\*}) + \cos\psi_{i^\*}\,(y - y_{i^\*})
\qquad (e > 0 \Rightarrow \text{vehicle left of the path})
$$

The **heading error** appears with two sign conventions in the code; both are
used below exactly as the corresponding controller uses them:

$$
\theta_e = \psi_{i^\*} - \psi \quad (\text{Stanley, simulator metrics}),
\qquad
\psi_e = \psi - \psi_{i^\*} \quad (\text{MPC}) .
$$

Because the reference is *sampled*, $\psi_{i^\*}$ and $\kappa_{i^\*}$ are
piecewise constant and jump by

$$
\Delta\psi_{i^\*} \approx \kappa\,\Delta s
$$

every time the nearest index advances — $0.0216\,\mathrm{rad} = 1.24^\circ$ on
the $R = 25\,\mathrm{m}$ bend. Controllers that use the local tangent directly
(Stanley, MPC) inherit this quantisation in their command; Pure Pursuit does
not (§6.2).

### 0.3 Reference speed profile

Curvature-limited, then smoothed by a backward pass so the vehicle brakes
*before* a corner (`assign_speed_profile`):

$$
v_{\mathrm{ref},i} = \mathrm{clamp}\!\left(\sqrt{\frac{a_{\mathrm{lat}}^{\max}}{|\kappa_i|}},\; v_{\min},\; v_{\max}\right),
\qquad
v_{\mathrm{ref},i-1} \leftarrow \min\!\left(v_{\mathrm{ref},i-1},\; \sqrt{v_{\mathrm{ref},i}^2 + 2 a_{\mathrm{lat}}^{\max}\Delta s}\right)
$$

with $a_{\mathrm{lat}}^{\max} = 1.5\,\mathrm{m/s^2}$, $v_{\min} = 1.5$,
$v_{\max} = 8.0\,\mathrm{m/s}$. On the benchmark route this gives
$8.0\,\mathrm{m/s}$ on the straights, $6.7$ on the $R = 30$ bend and
$6.1\,\mathrm{m/s}$ on the $R = 25$ bend.

### 0.4 Longitudinal law and actuator limits

Shared by all four controllers ([`controller.hpp`](../include/path_tracking/controller.hpp)):

$$
a = \mathrm{clamp}\big(k_p\,(v_{\mathrm{ref},i^\*} - v),\; -a_{\mathrm{dec}}^{\max},\; a_{\mathrm{acc}}^{\max}\big),
\qquad k_p = 1.2\,\mathrm{s^{-1}},\; a_{\mathrm{acc}}^{\max} = 1.5,\; a_{\mathrm{dec}}^{\max} = 3.0
$$

| | Pure Pursuit | Stanley | MPC | MPPI |
|---|---|---|---|---|
| $\lvert\delta\rvert \le$ | 0.6 rad (34.4°) | 0.6 rad | 0.6 rad | 0.6 rad |
| $\lvert\dot\delta\rvert \le$ | — (none) | 4 rad/s | 4 rad/s | 4 rad/s |

The rate limit corresponds to $4 \times 0.01 = 0.04\,\mathrm{rad} = 2.29^\circ$
of change per control cycle. Pure Pursuit needs none, which is itself a result
(§6.2).

---

## 1. Pure Pursuit

[`pure_pursuit.hpp`](../include/path_tracking/pure_pursuit.hpp) — geometric,
rear-axle referenced (Coulter, 1992).

### 1.1 Control law

Pick the first path sample at least $\ell_d$ ahead in arc length, measure the
angle $\alpha$ between the vehicle heading and the line to that point, and
steer along the circular arc that starts tangent to the vehicle and passes
through it:

$$
\alpha = \mathrm{wrap}\big(\operatorname{atan2}(y_{\mathrm{tgt}} - y,\; x_{\mathrm{tgt}} - x) - \psi\big),
\qquad
\delta = \arctan\!\left(\frac{2L\sin\alpha}{\ell_d}\right)
$$

with a speed-scheduled look-ahead distance

$$
\ell_d = \mathrm{clamp}(k_v\, v + \ell_{d,\min},\; \ell_{d,\min},\; \ell_{d,\max}),
\qquad k_v = 0.6\,\mathrm{s},\; \ell_{d,\min} = 3\,\mathrm{m},\; \ell_{d,\max} = 20\,\mathrm{m}
$$

### 1.2 Derivation

Let the arc through the target have radius $R_{pp}$. The triangle
(vehicle, arc centre, target) has two sides $R_{pp}$ and a chord $\ell_d$; the
angle between the chord and the vehicle heading is $\alpha$, and the heading is
perpendicular to the radius, so by the law of sines

$$
\frac{\ell_d}{\sin 2\alpha} = \frac{R_{pp}}{\sin\left(\tfrac{\pi}{2} - \alpha\right)}
\;\Longrightarrow\;
R_{pp} = \frac{\ell_d}{2\sin\alpha}
\;\Longrightarrow\;
\kappa_{\mathrm{cmd}} = \frac{2\sin\alpha}{\ell_d}
$$

Substituting into $\delta = \arctan(L\kappa)$ gives the law above. Written with
the lateral offset $y_{\mathrm{tgt}}^{\mathrm{veh}} = \ell_d \sin\alpha$ of the
target in the vehicle frame, the commanded curvature is

$$
\kappa_{\mathrm{cmd}} = \frac{2\,y_{\mathrm{tgt}}^{\mathrm{veh}}}{\ell_d^{\,2}}
$$

which is the form used in §1.4–1.5. The code uses `atan2(2L sin α, ld_eff)`
with $\ell_{d,\mathrm{eff}}$ the *actual* distance to the sample, which is
$\ge \ell_d$ because of the discrete spacing.

### 1.3 Closed-loop behaviour on a straight

Take the path as the $x$-axis, use $\psi_e = \psi - \psi_p$, and linearise for
small $e,\psi_e$. The target sits $\ell_d$ ahead on the path, so
$\alpha \approx -e/\ell_d - \psi_e$ and $\delta \approx (2L/\ell_d)\alpha$.
With $\dot e = v\sin\psi_e \approx v\psi_e$ and
$\dot\psi_e \approx (v/L)\delta$:

$$
\boxed{\;\ddot e + \frac{2v}{\ell_d}\dot e + \frac{2v^2}{\ell_d^{\,2}} e = 0\;}
$$

a second-order system with

$$
\omega_n = \frac{\sqrt2\,v}{\ell_d},
\qquad
\zeta = \frac{1}{\sqrt2} \approx 0.707
$$

Two consequences that explain the measured behaviour:

- **The damping ratio is constant** — independent of speed, look-ahead and
  wheelbase. Pure Pursuit is intrinsically well damped, which is why it needs
  no rate limiter and produces the smoothest steering in the benchmark
  (1.6 °/s RMS).
- **The bandwidth saturates with speed.** With $\ell_d = k_v v + \ell_{d,\min}$,
  $\omega_n \to \sqrt2/k_v = 2.36\,\mathrm{rad/s}$ as $v$ grows. Tracking
  accuracy therefore degrades with speed, and $k_v$ is the single knob that
  trades responsiveness ($\ell_d$ small) against stability and smoothness
  ($\ell_d$ large). At $v = 8\,\mathrm{m/s}$: $\ell_d = 7.8\,\mathrm{m}$,
  $\omega_n = 1.45\,\mathrm{rad/s}$, settling time $\approx 3/(\zeta\omega_n) \approx 2.9\,\mathrm{s}$.

### 1.4 Exactness on constant-curvature arcs

If the vehicle is *on* a circular path of curvature $\kappa$, the target lies on
the same circle at arc distance $\ell_d$, whose lateral offset in the vehicle
frame is $y^{\mathrm{veh}} = \kappa \ell_d^{\,2}/2$ exactly (chord geometry).
Then

$$
\kappa_{\mathrm{cmd}} = \frac{2}{\ell_d^{\,2}}\cdot\frac{\kappa \ell_d^{\,2}}{2} = \kappa
$$

so $e = 0$ is an exact equilibrium: **Pure Pursuit has no steady-state offset on
a circular arc**, regardless of $\ell_d$. This is confirmed by the trace — mean
cross-track error inside the constant-radius arcs is $-0.002\,\mathrm{m}$
($R = 200$), $-0.002$ ($R = 150$), $-0.006$ ($R = 30$) and $-0.012\,\mathrm{m}$
($R = 25$), i.e. essentially zero. The often-quoted "corner cutting by
$\kappa\ell_d^2/8$" applies to chasing a chord of a *polyline*, not to a
curvature-continuous reference.

### 1.5 Behaviour on clothoids — where the error actually appears

On a transition with linearly increasing curvature $\kappa(s) = c\,s$ (starting
from the vehicle), the target's lateral offset is
$y^{\mathrm{veh}} = \int_0^{\ell_d}\!\!\int_0^{u}\kappa\,\mathrm{d}\sigma\,\mathrm{d}u = c\,\ell_d^{\,3}/6$, hence

$$
\kappa_{\mathrm{cmd}} = \frac{2}{\ell_d^{\,2}}\cdot\frac{c\,\ell_d^{\,3}}{6} = \frac{c\,\ell_d}{3} = \kappa\!\left(s + \frac{\ell_d}{3}\right)
$$

**The look-ahead acts as a curvature preview of $\ell_d/3$** — roughly 2.5 m
here. The vehicle therefore turns in *before* the curve starts (cutting to the
inside) and straightens *before* the curve ends (drifting to the outside). The
trace shows exactly this on the tightest bend: peak $-0.198\,\mathrm{m}$
(inside) at $s = 914\,\mathrm{m}$ in the entry clothoid, and
$+0.163\,\mathrm{m}$ (outside) at $s = 961\,\mathrm{m}$ in the exit clothoid,
with the constant-radius arc between them tracked to within 0.01 m. Pure
Pursuit's worst error in this benchmark is a *transition* error, not a
curve error.

### 1.6 Parameters

| Field | Symbol | Default | Effect |
|---|---|---|---|
| `wheelbase` | $L$ | 2.7 m | Plant geometry; scales $\delta$ for a given curvature |
| `ld_gain` | $k_v$ | 0.6 s | Look-ahead per unit speed; sets the high-speed bandwidth $\sqrt2/k_v$ |
| `ld_min` | $\ell_{d,\min}$ | 3.0 m | Keeps the target ahead of the vehicle at low speed |
| `ld_max` | $\ell_{d,\max}$ | 20.0 m | Caps preview so tight bends are not cut |
| `steer_limit` | $\delta_{\max}$ | 0.6 rad | Saturation |

**Cost:** one windowed nearest search plus a forward walk — 0.35 µs/cycle, the
cheapest of the four.

---

## 2. Stanley

[`stanley.hpp`](../include/path_tracking/stanley.hpp) — geometric,
**front-axle** referenced (Stanford's 2005 DARPA Grand Challenge entry).

### 2.1 Control law

The reference point is the front axle,

$$
(x_f, y_f) = (x + L\cos\psi,\; y + L\sin\psi)
$$

and the nearest sample, heading error $\theta_e = \psi_{i^\*} - \psi$ and
cross-track error $e_f$ are all evaluated there:

$$
\boxed{\;\delta = k_\psi\,\theta_e - \arctan\!\left(\frac{k\,e_f}{k_{\mathrm{soft}} + v}\right)\;}
$$

The minus sign follows the repository's convention: $e_f > 0$ means the vehicle
is left of the path, which must be corrected by steering right. Defaults:
$k = 2.5$, $k_{\mathrm{soft}} = 1.0\,\mathrm{m/s}$, $k_\psi = 1.0$.

### 2.2 Convergence

The front wheel moves along the direction $\psi + \delta$ at speed
$v_f = v/\cos\delta \approx v$, so the lateral error evolves as
$\dot e_f = -v_f\sin(\theta_e - \delta)$. Substituting the control law
(with $k_\psi = 1$) cancels $\theta_e$ entirely:

$$
\dot e_f = -v\sin\!\left(\arctan\frac{k e_f}{k_{\mathrm{soft}} + v}\right)
        = -\frac{v\,k\,e_f}{\sqrt{(k_{\mathrm{soft}} + v)^2 + (k e_f)^2}}
$$

Two regimes:

- **Small error** ($k e_f \ll k_{\mathrm{soft}} + v$):
  $\dot e_f \approx -\dfrac{v k}{k_{\mathrm{soft}} + v}\,e_f$, i.e. exponential
  decay. For $v \gg k_{\mathrm{soft}}$ the time constant is $1/k = 0.4\,\mathrm{s}$
  — *speed-independent*, unlike Pure Pursuit. This is the property that made
  Stanley attractive at highway speed.
- **Large error**: $\dot e_f \to -v\,\mathrm{sign}(e_f)$ — the vehicle
  approaches the path at (almost) full speed, i.e. the $\arctan$ acts as a
  saturating, never-unstable gain. $V = \tfrac12 e_f^2$ gives $\dot V < 0$ for
  all $e_f \ne 0$, so convergence is global for this model.

$k_{\mathrm{soft}}$ exists to keep the gain finite as $v \to 0$; without it the
effective gain $k/v$ diverges and the controller chatters when stopping.

### 2.3 Steady-state offset on a curve — an exact result

Stanley zeroes the error **at the front axle**, and it does so exactly on a
constant-radius curve. On a circle of radius $R$ the required steering is
$\tan\delta = L/R_r$ where $R_r$ is the rear-axle radius; the vehicle heading is
tangent to the rear-axle circle while the path tangent at the front-axle
projection is rotated by $\arctan(L/R_r)$, so the heading term alone supplies
exactly the steering the curve needs:

$$
\theta_e = \arctan\!\left(\frac{L}{R_r}\right) = \delta_{\mathrm{required}}
\;\Longrightarrow\;
\arctan\!\left(\frac{k e_f}{k_{\mathrm{soft}} + v}\right) = 0
\;\Longrightarrow\;
e_f = 0
$$

With the front axle exactly on the path circle, the rear axle — the point the
metrics are measured at — trails on a circle of radius $\sqrt{R^2 - L^2}$, i.e.
**inside** the curve by

$$
\boxed{\;e_{\mathrm{rear}} = R - \sqrt{R^2 - L^2} \;\approx\; \frac{L^2\kappa}{2}\;}
$$

This is a *geometric* offset, not a missing feed-forward: adding a curvature
feed-forward would not remove it, because the front-axle error it regulates is
already zero. Measured against the trace:

| Bend | $\kappa$ [1/m] | Predicted $R - \sqrt{R^2 - L^2}$ [m] | Measured mean CTE [m] | ratio |
|---|---|---|---|---|
| $R = 200$ (left) | $+0.0050$ | $+0.0182$ | $+0.0169$ | 0.93 |
| $R = 150$ (left) | $+0.0067$ | $+0.0243$ | $+0.0225$ | 0.93 |
| $R = 30$ (right) | $-0.0333$ | $-0.1218$ | $-0.1155$ | 0.95 |
| $R = 25$ (right) | $-0.0400$ | $-0.1462$ | $-0.1391$ | 0.95 |

The residual 5–7 % is a discrete-time artefact of the 100 Hz loop: replaying the
same controller against an ideal circle reproduces the measured value
(0.01691 m at $R = 200$) and converges to the closed-form prediction as
$\Delta t \to 0$ (ratio 0.99 at $\Delta t = 1\,\mathrm{ms}$).

**Consequence:** Stanley's reported `cte_max` of 0.141 m *is* this steady-state
offset on the $R = 25$ bend, not a transient. Its error scales as $L^2\kappa/2$,
so it is negligible on highways ($\kappa \to 0$) and dominant in tight turns —
precisely the regime split seen in the report.

### 2.4 Phase-advance (lead) filter

An optional first-difference term damps oscillation on sharp curves:

$$
\delta^{\mathrm{out}}_n = (1 + c)\,\delta_n - c\,\delta_{n-1},
\qquad
C(z) = (1 + c) - c z^{-1},
\qquad c = 0.15
$$

Equivalently $\delta^{\mathrm{out}} \approx \delta + c\,\Delta t\,\dot\delta$: a
derivative action with a 1.5 ms derivative time. Note that `lead_state_` stores
the *pre-filter* command, so the filter differentiates the raw law, not its own
output. Set `lead_gain = 0` to disable. It contributes nothing in steady state
(constant $\delta$), so §2.3 is unaffected.

### 2.5 Parameters

| Field | Symbol | Default | Effect |
|---|---|---|---|
| `k_cross` | $k$ | 2.5 | Convergence rate; time constant $\approx 1/k$ at high speed |
| `k_soft` | $k_{\mathrm{soft}}$ | 1.0 m/s | Low-speed softening, avoids division by zero |
| `k_heading` | $k_\psi$ | 1.0 | Weight on heading error; **1.0 is what makes §2.3 exact** |
| `lead_gain` | $c$ | 0.15 | Phase advance on sharp curves |
| `steer_rate_limit` | $\dot\delta_{\max}$ | 4 rad/s | Actuation realism |

**Cost:** one windowed nearest search — 0.37 µs/cycle.

---

## 3. MPC (LTV / Riccati)

[`mpc.hpp`](../include/path_tracking/mpc.hpp) — finite-horizon linear-quadratic
control on the lateral error dynamics, re-solved every cycle.

### 3.1 Error state and prediction model

$$
z = \begin{bmatrix} e \\ \dot e \\ \psi_e \\ \dot\psi_e \end{bmatrix},
\qquad u = \delta,
\qquad
z_{k+1} = A z_k + B u_k
$$

initialised each cycle from the measured state, with $\dot e = v\sin\psi_e$
exact for the kinematic model and the yaw-rate error reset to zero:

$$
z_0 = \begin{bmatrix} e & v\sin\psi_e & \psi_e & 0\end{bmatrix}^{\!\top},
\qquad \psi_e = \psi - \psi_{i^\*}
$$

With $T = 0.05\,\mathrm{s}$ (`pred_dt`) and the speed frozen at
$v = \max(v_{\mathrm{meas}}, 0.5)$ over the horizon:

$$
A = \begin{bmatrix}
1 & T & 0 & 0\\
0 & 1 & Tv & 0\\
0 & 0 & 1 & T\\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
B = \begin{bmatrix} 0 \\ 0 \\ 0 \\ \dfrac{v}{L} \end{bmatrix}
$$

i.e. $\ddot e = v\psi_e$, $\ddot\psi_e = (v/L)\,\delta$. Two modelling choices
worth stating plainly:

- Steering enters through the **yaw-acceleration** channel, so $u$ is
  integrated twice into the heading. The resulting feedback is therefore of
  lead–lag character rather than a pure proportional gain on $\psi_e$.
- The curvature disturbance term ($-v\kappa$ in $\dot\psi_e$) is **not** in the
  model; the horizon assumes a locally straight reference. Curvature is handled
  entirely by the feed-forward of §3.5. In other words, this MPC does not
  preview the *future* path — the horizon shapes the feedback gains, and the
  only curvature information used is $\kappa_{i^\*}$ at the current nearest
  point.

The system is linear time-*varying* across cycles (because $A$, $B$ depend on
$v$), but time-invariant within a horizon.

### 3.2 Cost and solution

$$
J = \sum_{k=0}^{N-1}\left(z_k^\top Q z_k + R\,u_k^2\right) + z_N^\top Q_f z_N,
\qquad
Q = \mathrm{diag}(q_e, q_{\dot e}, q_\psi, q_{\dot\psi}),
\quad Q_f = \gamma Q
$$

minimised by the backward Riccati recursion — the exact solution of the
unconstrained problem, no QP solver needed:

$$
P_N = Q_f,\qquad
K_k = \frac{B^\top P_{k+1} A}{R + B^\top P_{k+1} B},\qquad
P_k = Q + A^\top P_{k+1} A - A^\top P_{k+1} B K_k
$$

Only the first gain is used (receding horizon): $u_0 = -K_0 z_0$. Defaults:
$q_e = 12$, $q_{\dot e} = 2$, $q_\psi = 8$, $q_{\dot\psi} = 2$, $R = 12$,
$\gamma = 4$, $N = 30$ ($1.5\,\mathrm{s}$ of horizon).

### 3.3 Resulting gains

Evaluating the recursion at $v = 8\,\mathrm{m/s}$:

$$
K_0 = \begin{bmatrix} 0.479 & 0.500 & 1.674 & 0.333\end{bmatrix}
$$

with closed-loop eigenvalues of $A - BK_0$ at $|\lambda| = 0.907, 0.907, 0.891, 0.319$,
i.e. a dominant time constant of $-T/\ln 0.907 = 0.52\,\mathrm{s}$ — about
twice as fast as Pure Pursuit's 0.97 s at the same speed, which is the
mechanism behind the 5× accuracy difference.

$N = 30$ is long enough to have converged onto the infinite-horizon (DARE)
solution:

| $N$ | 5 | 10 | 20 | **30** | 60 | $\infty$ |
|---|---|---|---|---|---|---|
| $K_{0,1}$ (gain on $e$) | 0.012 | 0.139 | 0.410 | **0.479** | 0.483 | 0.483 |

So this controller is, numerically, a well-tuned LQR plus feed-forward; the
receding-horizon machinery buys re-linearisation at the current speed rather
than genuine preview. Shortening `horizon` below ~20 is what actually changes
the behaviour (and degrades it).

### 3.4 Why the weights behave the way they do

$R/q_e$ sets the aggressiveness: the loop gain scales roughly as
$\sqrt{q_e/R}$, so doubling `r_steer` smooths the command by ~30 % and widens
the error by the same order. $q_{\dot e}$ and $q_{\dot\psi}$ add damping
(they penalise the rates directly). $\gamma = Q_f/Q$ mainly matters for short
horizons; at $N = 30$ its influence is below 1 %.

### 3.5 Curvature feed-forward

$$
\delta = \mathrm{clamp}\big(\underbrace{\arctan(L\kappa_{i^\*})}_{\text{feed-forward}} \;\underbrace{-\,K_0 z_0}_{\text{feedback}}\big),
\qquad
|\dot\delta| \le 4\,\mathrm{rad/s}
$$

The feed-forward is *exact* for the kinematic model (§0.1), and because MPC
regulates the error at the rear axle — the same point the metrics use — it has
**no geometric steady-state offset**, unlike Stanley (§2.3). That, not preview,
is the main reason MPC reaches 0.008 m RMS.

### 3.6 A numerical artefact worth knowing

MPC's steering rate RMS (46.6 °/s, second highest of the four) does not come
from the LQR. On the $R = 25\,\mathrm{m}$ bend the command runs in a period-9
sawtooth: the nearest index advances every $\Delta s / v = 0.54/6.13 = 0.088\,\mathrm{s}$
$= 9$ control cycles, and each advance steps the reference heading by
$\kappa\Delta s = 0.0216\,\mathrm{rad}$. Through the gains this is a command
jump of order

$$
\Delta\delta \approx (K_{0,3} + v\,K_{0,2})\,\kappa\,\Delta s \approx 0.12\,\mathrm{rad} \approx 7^\circ
$$

which the 4 rad/s rate limit stretches over two cycles (observed swing: 3.7° →
8.3°, i.e. 4.6° peak-to-peak, repeating every 9 cycles). The mean of that
sawtooth is slightly below $\arctan(L\kappa) = 6.15^\circ$, which also explains
the small residual $+0.028\,\mathrm{m}$ (outside) offset on that bend. Fixes
would be interpolating the reference between samples, or low-pass filtering
$\psi_{i^\*}$ — neither is implemented, and it is left visible here because it
is a real and common failure mode of map-based controllers.

### 3.7 Parameters and cost

| Field | Symbol | Default | Effect |
|---|---|---|---|
| `horizon` | $N$ | 30 | Steps; ≥20 needed for DARE convergence |
| `pred_dt` | $T$ | 0.05 s | Prediction step; $NT = 1.5\,\mathrm{s}$ horizon |
| `q_cte` / `q_yaw` | $q_e$ / $q_\psi$ | 12 / 8 | Error weights |
| `q_cte_rate` / `q_yaw_rate` | $q_{\dot e}$ / $q_{\dot\psi}$ | 2 / 2 | Damping |
| `r_steer` | $R$ | 12 | Control effort; higher = smoother, less accurate |
| `qf_scale` | $\gamma$ | 4 | Terminal weight |

**Cost:** $O(N n^3)$ with $n = 4$ — 30 iterations of small dense matrix
algebra, 1.52 µs/cycle. Eigen is used only here.

---

## 4. MPPI

[`mppi.hpp`](../include/path_tracking/mppi.hpp) — Model Predictive Path
Integral (Williams et al., 2016/2017): sampling-based receding-horizon control.

### 4.1 Idea

Instead of solving for the optimum, sample $K$ noisy control sequences, roll
each one out through the nonlinear model, and take a **cost-weighted average**.
The path-integral / free-energy result states that the optimal control
distribution is a reweighting of the sampling distribution,

$$
q^\*(\tau) \;\propto\; \exp\!\left(-\frac{S(\tau)}{\lambda}\right) p(\tau)
$$

so the next control sequence is the importance-weighted mean
$u^\* = \mathbb{E}_{q^\*}[u]$, estimated from the samples. $\lambda$ is a
temperature: $\lambda \to 0$ recovers "take the best rollout" (greedy, high
variance), $\lambda \to \infty$ averages everything and the update vanishes.

### 4.2 Algorithm, as implemented

Per cycle, with a nominal sequence $\bar u \in \mathbb{R}^N$ carried across
cycles:

1. **Warm start (shift):** $\bar u_i \leftarrow \bar u_{i+1}$ for $i < N-1$.
2. **Sample** $K$ perturbations $\varepsilon^{(k)}_i \sim \mathcal N(0, \sigma^2)$ and form
   $u^{(k)}_i = \mathrm{clamp}(\bar u_i + \varepsilon^{(k)}_i, \pm\delta_{\max})$.
3. **Roll out** each sequence at constant speed through the kinematic model
   with step $T$:
   $$x \mathrel{+}= vT\cos\psi,\quad y \mathrel{+}= vT\sin\psi,\quad \psi \mathrel{+}= \frac{v\tan u_i}{L}T$$
4. **Score** each rollout:
   $$S_k = \sum_{i=0}^{N-1}\Big(w_e\,e_i^2 + w_\psi\,\theta_{e,i}^2 + w_\delta\,{u^{(k)}_i}^2\Big)$$
5. **Weight** (softmax, shifted by $S_{\min}$ for numerical stability):
   $$w_k = \frac{\exp\!\big(-(S_k - S_{\min})/\lambda\big)}{\sum_{j}\exp\!\big(-(S_j - S_{\min})/\lambda\big)}$$
6. **Update and apply:**
   $$\bar u_i \leftarrow \mathrm{clamp}\Big(\bar u_i + \sum_{k} w_k\,\varepsilon^{(k)}_i\Big),
   \qquad \delta = \mathrm{rate\_limit}(\bar u_0)$$

Defaults: $K = 120$ samples, $N = 20$ steps, $T = 0.05\,\mathrm{s}$ (1.0 s
horizon), $\sigma = 0.08\,\mathrm{rad}$, $\lambda = 1.0$, $w_e = 8$,
$w_\psi = 3$, $w_\delta = 0.3$, fixed RNG seed 12345 for reproducibility.

### 4.3 Differences from the canonical formulation

Worth knowing before porting this code elsewhere:

- The information-theoretic weight of Williams et al. includes a control term
  $\frac{\gamma}{\lambda}\bar u^\top \Sigma^{-1}\varepsilon$ from the importance-sampling
  correction; here the weight is a plain softmax over the state cost, with
  control effort entering through $w_\delta u^2$ instead. This is the common
  "vanilla MPPI" simplification and biases the update slightly, but keeps it
  well behaved for quadratic costs.
- The rollout holds the **speed constant** — longitudinal dynamics are not
  simulated inside the sampler.
- Sequences are clamped inside the rollout, so the sampling distribution is
  truncated near the steering limits.

### 4.4 Sample-efficiency and the noise floor

The estimator quality is governed by the effective sample size

$$
\mathrm{ESS} = \frac{1}{\sum_k w_k^2} \in [1, K]
$$

When the cost spread is large compared with $\lambda$, a handful of rollouts
carry all the weight and the update behaves like a small random walk of
standard deviation $\approx \sigma/\sqrt{\mathrm{ESS}}$ per cycle. The measured
steering-rate RMS of 174.8 °/s corresponds to
$\approx 0.030\,\mathrm{rad}$ of change per cycle, i.e.
$\mathrm{ESS} \sim \mathcal{O}(10)$ out of $K = 120$ — this jitter is what shows
up as the visibly noisy MPPI trace ($\pm 0.05\,\mathrm{m}$ on straights) and its
0.384° heading-error RMS, the worst of the four. Raising $\lambda$ or $K$, or
lowering $\sigma$, trades that noise against exploration and compute.

### 4.5 Where the compute goes

Per control cycle the sampler performs $K \times N = 2400$ rollout steps, and
*each* step runs a windowed nearest search over up to $2W = 400$ path samples:

$$
120 \times 20 \times 400 \approx 9.6\times10^5 \ \text{distance evaluations / cycle}
$$

At the ~0.6 ns per evaluation implied by the Pure Pursuit timing, that alone
accounts for ≈ 570 µs — and the measured cost is 568 µs/cycle. **MPPI's cost
here is the nearest-point search, not the dynamics or the exponentials.**
Shrinking the rollout window, or tracking the index incrementally, would cut it
by an order of magnitude before any parallelisation. The algorithm is also
embarrassingly parallel over $k$ (the reference build is single-threaded).

### 4.6 When MPPI is worth it

Nothing in this benchmark exercises what MPPI is for: the cost is quadratic and
differentiable, the model is smooth, and there are no obstacles or discrete
decisions. Its strengths — non-convex, non-differentiable, or discontinuous
costs (collision indicators, drivable-area masks, contact dynamics) — cannot be
expressed at all in the LQR formulation of §3, which is why it pays ~1600× the
per-cycle cost of the geometric methods here for equivalent accuracy.

---

## 5. ACC longitudinal law

[`acc_controller.hpp`](../include/path_tracking/acc_controller.hpp) — used by
the cut-in demo, with Pure Pursuit steering unchanged.

### 5.1 Law

Constant time-headway spacing policy plus proportional feedback on both the
relative speed and the gap error:

$$
g_{\mathrm{des}} = \max(g_{\min},\; h\,v_{\mathrm{ego}}),
\qquad
a = \mathrm{clamp}\Big(k_{\Delta v}\,(v_{\mathrm{lead}} - v_{\mathrm{ego}}) + k_g\,(g - g_{\mathrm{des}}),\; -a^{\max}_{\mathrm{dec}},\; a^{\max}_{\mathrm{acc}}\Big)
$$

with $h = 2.0\,\mathrm{s}$, $g_{\min} = 5\,\mathrm{m}$,
$k_{\Delta v} = 1.2$, $k_g = 0.4$, falling back to the path speed law
(§0.4) when no lead is within `detection_range` = 80 m.

### 5.2 Closed-loop response

Let $\varepsilon = g - h v_{\mathrm{ego}} - g_0$ be the spacing error and
$\Delta v = v_{\mathrm{lead}} - v_{\mathrm{ego}}$. With a constant-speed lead
and no saturation:

$$
\begin{bmatrix}\dot\varepsilon \\ \Delta\dot v\end{bmatrix}
=
\begin{bmatrix} -h k_g & 1 - h k_{\Delta v} \\ -k_g & -k_{\Delta v}\end{bmatrix}
\begin{bmatrix}\varepsilon \\ \Delta v\end{bmatrix}
\;\Longrightarrow\;
s^2 + (k_{\Delta v} + h k_g)\,s + k_g = 0
$$

A clean result: the natural frequency depends only on the gap gain and the
damping only on the sum $k_{\Delta v} + h k_g$:

$$
\omega_n = \sqrt{k_g} = 0.63\,\mathrm{rad/s},
\qquad
\zeta = \frac{k_{\Delta v} + h k_g}{2\sqrt{k_g}} = 1.58
$$

i.e. overdamped, with poles at $-0.225$ and $-1.775\,\mathrm{s^{-1}}$. The slow
pole ($\tau = 4.4\,\mathrm{s}$) governs gap recovery, so the demo settles to the
lead speed in roughly 13 s after the cut-in — the third phase visible in the
plot. Increasing $k_g$ speeds up recovery as $\sqrt{k_g}$ but reduces $\zeta$;
increasing $h$ adds damping at the cost of a larger gap.

---

## 6. Side-by-side summary

### 6.1 Structural comparison

| | Pure Pursuit | Stanley | MPC | MPPI |
|---|---|---|---|---|
| Reference point | rear axle | **front axle** | rear axle | rear axle |
| Uses lateral error directly | no (implicit, via target) | yes | yes | yes (in the cost) |
| Uses heading error | implicit | yes | yes | yes |
| Curvature handling | preview $\ell_d/3$ (§1.5) | none | feed-forward $\arctan(L\kappa)$ | implicit in the rollout |
| Preview of the future path | geometric only | none | none (see §3.1) | yes, $NT = 1.0\,\mathrm{s}$ |
| Optimality | none | none | exact LQ optimum | stochastic estimate |
| Steady-state error on an arc | 0 (§1.4) | $L^2\kappa/2$ (§2.3) | 0 (§3.5) | ~0 (noisy) |
| Tuning knobs | 1 ($k_v$) | 2 ($k$, $k_{\mathrm{soft}}$) | 6 (weights, $N$) | 6 ($\sigma$, $\lambda$, $K$, $N$, weights) |
| Cost per cycle | 0.35 µs | 0.37 µs | 1.52 µs | 568 µs |
| Dominant cost | nearest search | nearest search | Riccati, $O(Nn^3)$ | $KN$ nearest searches |

### 6.2 Why the measured numbers come out as they do

| Observation | Explanation |
|---|---|
| Pure Pursuit has the smoothest steering (1.6 °/s) with **no** rate limiter | $\zeta = 1/\sqrt2$ regardless of tuning (§1.3), and its input is the direction to a point 7.8 m away, which low-pass filters the 0.54 m map quantisation instead of differentiating it |
| Pure Pursuit has the largest peak error (0.198 m) | The $\ell_d/3$ curvature preview turns in early on the entry clothoid (§1.5) — a transition error, not a curve error |
| Stanley's peak (0.141 m) equals its offset on the $R = 25$ bend | Geometric front-axle-vs-rear-axle offset $L^2\kappa/2$ (§2.3), verified to within 5–7 % on all four bends |
| MPC is 5× more accurate (0.008 m RMS) | Exact curvature feed-forward + a 0.52 s closed-loop time constant, regulating the same point the metric measures (§3.3, §3.5) |
| MPC's steering rate is high (46.6 °/s) | Period-9 sawtooth from nearest-point quantisation amplified by $K_{0,2}v + K_{0,3}$ (§3.6), not the LQR itself |
| MPPI's trace is noisy (174.8 °/s, 0.384° heading RMS) | Monte-Carlo update noise $\sigma/\sqrt{\mathrm{ESS}}$ with $\mathrm{ESS} \sim 10$ (§4.4) |
| MPPI costs 568 µs/cycle | $9.6\times10^5$ nearest-search distance evaluations per cycle (§4.5) |

---

## 7. References

1. R. C. Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*,
   CMU-RI-TR-92-01, 1992.
2. G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, S. Thrun, *Autonomous Automobile
   Trajectory Tracking for Off-Road Driving: Controller Design, Experimental
   Validation and Racing*, ACC 2007. (Stanley controller and its convergence
   proof.)
3. S. Thrun et al., *Stanley: The Robot that Won the DARPA Grand Challenge*,
   Journal of Field Robotics, 2006.
4. J. M. Snider, *Automatic Steering Methods for Autonomous Automobile Path
   Tracking*, CMU-RI-TR-09-08, 2009. (Comparison of geometric and
   model-based trackers; the lateral-error state model of §3.1.)
5. G. Williams, A. Aldrich, E. Theodorou, *Model Predictive Path Integral
   Control: From Theory to Parallel Computation*, JGCD, 2017; and G. Williams
   et al., *Aggressive Driving with Model Predictive Path Integral Control*,
   ICRA 2016.
6. B. D. O. Anderson, J. B. Moore, *Optimal Control: Linear Quadratic Methods*,
   1990. (Riccati recursion and its convergence to the DARE solution.)
