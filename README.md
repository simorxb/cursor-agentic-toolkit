# MATLAB and Simulink Agentic Toolkit - Control Design Experiments

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=simorxb/cursor-agentic-toolkit)

## Summary
This repository collects small control-design studies used to explore what the MATLAB and Simulink Agentic Toolkit can do when an agent is asked to design, run, and check MATLAB against explicit requirements. Each study is a mini-project in its own folder. The first one, `design-controller`, designs a controller for an unstable second-order plant and checks stability, sensitivity margin, rise time, and overshoot in MATLAB.

## Project Overview
The toolkit gives an agent a local MATLAB session, including the Control System Toolbox and the Robust Control Toolbox. A mini-project is a folder whose script the agent writes and then executes, so the result is a design that has been checked numerically.

New studies are added as new folders and as new parts of this README. Existing parts stay as they are.

### Key Features
- **One folder per study**: `design-controller` is the first mini-project.
- **Checked requirements**: the script asserts stability, sensitivity margin, rise time, and overshoot before it finishes.
- **Closed-loop figures**: running the script plots the step response, the control effort, and the sensitivity magnitude.

---

# Part 1 – Controller Design for an Unstable Plant

## Summary
This part stabilizes the plant $P(s) = 1/(s^2 - 0.1s + 1)$ with a two-degree-of-freedom PID. The controller is obtained by pole placement on the plant plus an integrator, with a filtered derivative, and `design-controller/design_controller.m` checks the closed loop in MATLAB.

## Project Overview
The task was to design a controller that stabilizes this plant and meets three limits: sensitivity margin greater than 0.5, 10% to 90% rise time under 1 s, and overshoot under 10%.

The open-loop poles are a complex pair in the right half-plane, at $0.05 \pm j0.999$, with magnitude 1. Gain below the level needed to encircle the critical point leaves the loop unstable, so the crossover has to sit above that magnitude. The same loop also has to keep the step response inside the rise-time and overshoot limits.

## Design Process

### Sensitivity margin
A peak of $|S|$ below 0.5 is not a usable reading of the requirement. This loop is strictly proper, so $S(\infty) = 1$ and $\|S\|_\infty \ge 1$. The requirement was read as a modulus margin, the distance from the Nyquist plot of the loop transfer $L$ to the point $-1$:

$$
m_s = \min_{\omega} |1 + L(j\omega)| = \frac{1}{\|S\|_\infty}
$$

Where:
- $m_s$ is the sensitivity margin.
- $\omega$ is the frequency in rad/s.
- $L$ is the loop transfer.
- $S = 1/(1+L)$ is the sensitivity function.
- $m_s > 0.5$ means $\|S\|_\infty < 2$.

### Steps that were tried
1. **Filtered PID with `systune`.** A PID with a derivative filter was tuned against a step-tracking goal and a hard limit near $\|S\|_\infty < 1.7$. The closed loop was stable, and the three limits were formally met: margin about 0.94, rise time about 0.96 s, overshoot about 8%. The proportional and integral gains collapsed, the normalized tracking cost stayed well above 1, and the 2% settling time was about 32 s. That design was discarded.
2. **Pole placement with an integrator.** The plant state $[y,~\dot{y}]$ was augmented with $\xi$, where $\dot{\xi} = r - y$. `place` returns the gain for $u = -Kx$. In the regulator model $\dot{\xi} = -y$, so the integral gain is the negative of the third entry of that gain.
3. **One-degree-of-freedom PID.** Using those gains as $u = C(s)(r - y)$ differentiates the reference step. A sweep of natural frequency and damping, all with a derivative filter of 0.02 s, produced overshoot from about 17% to about 23%. The rise time was short because of that kick, and the overshoot limit was missed in every case.
4. **Two-degree-of-freedom PID.** Proportional and derivative action were applied to $y$ only, and the integrator was left on the tracking error. That is the control law the placement designs. The target poles were a dominant pair with damping 0.7 and natural frequency 3.2 rad/s, plus a real pole at $-8$. The derivative filter time constant is 0.02 s, about a decade above that natural frequency, which makes the controller proper. The filter moves the dominant poles from about $-2.24 \pm j2.29$ to about $-2.12 \pm j2.06$. The script checks the filtered closed loop.

## Controller Parameters
The control law is

$$
u = \frac{K_i}{s}(r - y) - K_p y - \frac{K_d s}{\tau s + 1} y
$$

Where:
- $u$ is the plant input.
- $r$ is the reference.
- $y$ is the measured output.
- $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains.
- $\tau$ is the derivative-filter time constant.
- $s$ is the Laplace variable.

- **Proportional gain** $K_p$: 45.08
- **Integral gain** $K_i$: 81.92
- **Derivative gain** $K_d$: 12.58
- **Derivative filter** $\tau$: 0.02 s
- **Target damping**: 0.7
- **Target natural frequency**: 3.2 rad/s
- **Additional closed-loop pole**: $-8$

## Results and Performance
`design-controller/design_controller.m` computes the gains, builds the closed loop, and asserts the four requirements. A unit-step run in MATLAB gave:

### Highlights
- **Stability**: the closed loop is stable.
- **Sensitivity margin**: 0.822, against a requirement above 0.5.
- **Rise time**: 0.754 s from 10% to 90%, against a requirement under 1 s.
- **Overshoot**: 3.83%, against a requirement under 10%.
- **Settling time**: 2.08 s to the 2% band.
- **Phase margin**: 59.5 deg at 13.1 rad/s.
- **Gain reduction**: the loop stays stable for gain factors above 0.127. A further reduction removes the encirclements the unstable plant requires.
- **Peak control effort**: about 5.0 on a unit step.

## Key Takeaways
- The sensitivity margin above 0.5 is a modulus margin, $1/\|S\|_\infty > 0.5$, because $\|S\|_\infty$ cannot fall below 1 for this loop.
- A `systune` design can meet the three numbers and still be a poor controller. Here the price was a settling time of about 32 s.
- The same pole-placement gains miss the overshoot limit when the derivative acts on the reference step. Applying proportional and derivative action to the measurement, and integral action to the error, meets the limit without giving up the sensitivity margin.

---

# Part 2 – Kalman Filter and Exponential Moving Average

## Summary
This part works out when a scalar Kalman filter is an exponential moving average, and where that stops. Start-up is one place. A ramp is the other. `kalman-filter-vs-ema/kalman_vs_ema.m` runs the filters in base MATLAB, checks the steady gain with `idare` from the Control System Toolbox, and writes four portrait figures.

## Project Overview
A random walk measured in white noise has a constant steady-state gain. With that gain the Kalman update is an exponential moving average, same estimate on every step. While the variance is still large the gain is larger, so the Kalman filter moves first and the moving average catches up later. On a ramp both of those filters lag by the same amount. A second state for velocity removes the lag. The lag belongs to the random-walk model.

### Key Features
- **One noise draw**: seed 1, shared by the Kalman filter and the moving average.
- **Three gain checks**: closed form, Riccati iteration, and `idare`.
- **Ramp**: the same moving average, the one-state filter, and a constant-velocity filter.
- **Portrait figures**: 1080 by 1350, one message each.

## Steady-State Update
The scalar random walk is

$$
x_{k+1} = x_k + w_k, \qquad y_k = x_k + v_k
$$

Where:
- $x_k$ is the state.
- $y_k$ is the measurement.
- $w_k \sim N(0,~Q)$ is the process noise.
- $v_k \sim N(0,~R)$ is the measurement noise.

$F = 1$, so the predict step does not move the estimate. The update is

$$
\hat{x} = \hat{x} + K(y - \hat{x}) = (1 - K)\hat{x} + Ky
$$

In steady state the predicted variance is the positive root of $P^2 - QP - QR = 0$:

$$
P^{-}_{\infty} = \frac{Q + \sqrt{Q^2 + 4QR}}{2}, \qquad K_{\infty} = \frac{P^{-}_{\infty}}{P^{-}_{\infty} + R}
$$

The update is then an exponential moving average with weight $\alpha = K_{\infty}$.

## Simulation Setup
Each case calls `rng(1, 'twister')` before it draws noise, so either case reproduces on its own. The random walk draws process noise and then measurement noise. The ramp resets the seed and draws measurement noise only.

- **Process noise** $Q$: 0.01
- **Measurement noise** $R$: 1
- **Steps**: 2000, indexed from $k = 0$
- **Sample time**: 1
- **Truth at** $k = 0$: 5 for the random walk, and $x_k = 0.05k$ for the ramp
- **Filter start**: estimate 0, variance $P_0 = 10$
- **Velocity model**: $F = [1,~T_s;~0,~1]$, $H = [1,~0]$, process noise $10^{-6}$ times $[T_s^3/3,~T_s^2/2;~T_s^2/2,~T_s]$, $P_0 = 10I$, velocity prior 0

### How to run
From the repository folder:

```matlab
run(fullfile('kalman-filter-vs-ema', 'kalman_vs_ema.m'))
```

The script writes the PNGs under `kalman-filter-vs-ema/figures/` and stops if a gain check or a ramp check fails.

## Results and Performance
MATLAB R2024a, seed 1. The gain numbers do not depend on the seed.

### Highlights
- **Steady gain** $K_{\infty}$: 0.095125. The closed form, the Riccati recursion (158 iterations from $P_0$), and `idare(1, 1, Q, R)` agree through 12 digits. For this scalar filter `idare` returns the predicted variance, and its gain output is $K_{\infty}$.
- **Gain at** $k = 0$: 0.909
- **Within** $10^{-3}$ of $K_{\infty}$: from $k = 26$. At $k = 50$, $K = 0.095132$.
- **Constant-gain filter against the moving average**: maximum absolute difference $1.24 \times 10^{-14}$, on the whole record.
- **Time-varying filter against the moving average**: $2.68 \times 10^{-2}$ for $k \ge 50$, $8.25 \times 10^{-9}$ for $k \ge 200$, $1.24 \times 10^{-14}$ for $k \ge 1000$. The gap stays under $10^{-9}$ from $k = 222$.
- **Theoretical ramp lag** $v(1-\alpha)/\alpha$: 0.4756
- **Simulated lag** for $k \ge 1000$, truth minus estimate: 0.394 for the moving average and 0.394 for the one-state filter. Those two estimates differ by at most $8.53 \times 10^{-14}$ on that window.
- **Two-state mean error** for $k \ge 1000$, estimate minus truth: $+0.083$

### What moved with the seed
A separate check, different seed, had an EMA lag near 0.46 and a two-state mean error near $-0.02$. Seed 1 does not reproduce those two averages. Over $k \ge 1000$ the measurements sit 0.080 above the ramp. Subtract that mean from the theoretical lag and the result is 0.396, against a simulated lag of 0.394. The two-state error sits 0.003 above the same noise mean. Shorter slices of the record do the same thing: the moving-average lag tracks $v(1-\alpha)/\alpha$ minus the noise mean of the slice, and the velocity filter tracks the noise mean. A simulated lag quoted to two digits is the noise draw.

$1/\alpha = 10.5$ steps is the moving-average time constant. In the first panel the orange curve is still closing on the truth out near step 40.

"After the transient" also depends on the window. The gain has arrived by about step 50. The estimate gap left by the early, larger gains is still 0.027 there, and it has fallen below $10^{-9}$ by step 222. The constant-gain filter matches the moving average from step 0.

### Gain against the noise ratio
$K_{\infty}$ depends on $Q/R$ only.

- **$Q/R = 10^{-4}$**: $K_{\infty} = 0.00995$
- **$Q/R = 10^{-3}$**: $K_{\infty} = 0.03113$
- **$Q/R = 10^{-2}$**: $K_{\infty} = 0.095125$
- **$Q/R = 10^{-1}$**: $K_{\infty} = 0.2702$
- **$Q/R = 1$**: $K_{\infty} = 0.6180$

![After start-up, the Kalman filter is an EMA](kalman-filter-vs-ema/figures/random_walk_kf_and_ema.png)

![The gain settles at the EMA weight](kalman-filter-vs-ema/figures/kalman_gain.png)

![Random-walk model lags a ramp. Velocity model tracks it.](kalman-filter-vs-ema/figures/ramp_tracking_error.png)

![Q/R picks the EMA weight](kalman-filter-vs-ema/figures/alpha_from_qr.png)

## Files
- **`kalman-filter-vs-ema/kalman_vs_ema.m`**: random walk, ramp, gain checks, and the four figures.

## Key Takeaways
- With $F = 1$, a constant Kalman gain is an exponential moving average, $\alpha = K_{\infty}$, to roundoff.
- The time-varying filter uses a large gain at start-up. It matches the moving average after that gain has settled and the early difference has died out, here by about step 220 for a difference under $10^{-9}$.
- On a ramp the one-state filter lags by $v(1-\alpha)/\alpha$, the same lag as the moving average. The missing state is velocity.
- The constant-velocity filter tracks the ramp. Over a finite window its average error follows the average of the measurement noise.

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
