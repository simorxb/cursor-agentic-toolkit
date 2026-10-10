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

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
