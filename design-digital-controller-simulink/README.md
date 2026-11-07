# Digital Stabilizing Controller (Simulink)

Discrete controller for the unstable plant

$$
P(s) = \frac{1}{s^2 - 0.1\,s + 1}
$$

Sample time $T_s = 0.1\,\mathrm{s}$. The final deliverable is a closed-loop Simulink model plus MATLAB scripts that retune and re-check every requirement after each design change.

## Summary

This project designs a digital feedback controller that stabilizes $P(s)$, meets tracking and robustness specs, and runs in Simulink. The controller ended as a discrete 2-DOF PID with filtered derivative (PIDF2). The interesting part is not the final gains. It is the sequence of requirement changes and the verification loop used each time so older specs never silently broke.

## Final requirements (all must pass)

| Requirement | Target | Final result |
|-------------|--------|--------------|
| Closed-loop stability | Stable | Yes |
| Sensitivity margin $S_m = 1/M_s$ | > 0.5 | ≈ 0.67 |
| Rise time | < 1 s | ≈ 0.59 s |
| Overshoot | < 10% | ≈ 8.1% |
| Settling envelope | $\|y-1\| \le 2\%$ for all $t \ge 2$ s | Yes |
| Digital sample time | 0.1 s | 0.1 s |
| Control effort | No large step kick on $u$ | peak $\|u\|$ ≈ 10 |

Re-check everything with:

```matlab
run scripts/verify_requirements.m
```

That script redesigns (or regenerates) the controller, asserts analysis metrics, runs the Simulink model, asserts the same metrics on logged $y$, and plots analysis vs Simulink for $y$ and $u$.

## Design story

### 1. Baseline brief

Start from an unstable plant (poles near $+0.05 \pm j$). Required:

- stabilize the loop
- $S_m > 0.5$
- rise time < 1 s
- overshoot < 10%
- discrete controller at $T_s = 0.1$ s
- closed-loop demo in Simulink

Plant used for tuning: $P_d(z) = \mathrm{c2d}(P, 0.1, \mathrm{zoh})$. Continuous $P(s)$ stays in the Simulink plant block so the demo is hybrid (discrete $u$, continuous plant).

### 2. First working design: 1-DOF PIDF + prefilter

`pidtune` on a discrete PIDF gave a fast loop, but overshoot sat above 10% at useful bandwidths. Raising phase margin fixed overshoot and then settling became very slow.

Compromise that first passed the original four specs:

- feedback: discrete PIDF, target phase margin 60°
- reference prefilter: $F(z)=\mathrm{c2d}(1/(0.3s+1), 0.1, \mathrm{tustin})$

Architecture then:

```
r → PrefFilter → Σ → Discrete PIDF → Plant P(s) → y
                  ↑−                         │
                  └───────────────────────────┘
```

Verification at that stage: frequency-domain $S_m$ on $L=C P_d$, stepinfo on $T = F \cdot \frac{C P_d}{1+C P_d}$, then a Simulink unit-step MIL with the same thresholds. Typical numbers: $t_r \approx 0.56$ s, $M_p \approx 3.8\%$, $S_m \approx 0.80$.

### 3. New requirement: no late undershoot

The step response dipped again around 2.5 s. New acceptance rule:

> $|y(t)-1| \le 0.02$ for every sample with $t \ge 2$ s

The 1-DOF + prefilter family could not meet that envelope without giving up other specs (survey in `scripts/retune_settling.m`). Discrete **PIDF2** (2-DOF PID with setpoint weights $b$, $c$) could.

Switched architecture to:

```
r -----> Discrete PIDF 2DOF -----> Plant P(s) -----> y
              ^                                      |
              +--------------------------------------+
```

First PIDF2 candidate used `pidtune(..., 'PIDF2')` at 50° phase margin. Analysis looked excellent. Early Simulink checks failed until the model on disk actually contained the 2-DOF block (an older 1-DOF path was still being simulated). After rebuilding `models/digital_stabilizing_controller.slx` with `Discrete PID Controller (2DOF)`, analysis and Simulink matched: $t_r \approx 0.32$ s, $M_p \approx 2.3\%$, $S_m \approx 0.70$, and $y$ stayed inside $\pm 2\%$ after 2 s.

Verification update: `verify_requirements.m` gained an explicit settling-band assert on both the discrete analysis response and `y_sim`.

### 4. New requirement: smoother $u$

Plotting control effort showed the problem immediately. Peak $|u|$ was about 106 on the unit step. Mostly derivative kick through the reference channel.

Softening pass (`scripts/soften_u.m`, then locked into `design_controller.m`):

- keep `pidtune` PIDF2 at 50° as the nominal loop shape
- scale $K_p \times 0.75$, $K_d \times 0.65$
- set $T_f = 0.08$ (heavier D filter)
- set $b = 0.9$, $c = 0.05$ (almost no D on $r$)

Then re-run the full checklist. Final numbers: peak $|u| \approx 10$, $t_r \approx 0.59$ s, $M_p \approx 8.1\%$, $S_m \approx 0.67$, settling band still pass. All previous requirements still held.

Verification update: the same script now plots two subplots ( $y$ and $u$ ), analysis vs Simulink, with thick $\pm 2\%$ guides on $y$.

## How requirements were protected after every change

Same loop every time a new ask landed:

1. **Encode the new rule** in `design_controller.m` and `verify_requirements.m` (assert, not just a comment).
2. **Search candidates** with a sweep script when the obvious knob was not enough (`explore_designs.m`, `refine_design.m`, `retune_settling.m`, `soften_u.m`).
3. **Pick a design that passes every current assert**, not only the newest one.
4. **Export gains** to `scripts/init_controller_params.m` and `controller_design.mat`.
5. **MIL in Simulink** via `Simulink.SimulationInput` on `digital_stabilizing_controller.slx`.
6. **Compare analysis vs logged signals** so a discrete-only win cannot hide a hybrid miss.

If Simulink disagreed with analysis, the model wiring was treated as suspect before the tuning was blamed. That is what caught the stale 1-DOF model during the settling-envelope work.

## Architecture (current)

Discrete 2-DOF PIDF driving continuous $P(s)$:

```
r -----> Discrete PIDF 2DOF -----> Plant P(s) -----> y
              ^                                      |
              +--------------------------------------+
```

- **Controller block**: `Discrete PID Controller (2DOF)` at $T_s = 0.1$ s, Forward Euler I/D
- **Plant block**: continuous Transfer Fcn `1/(s^2-0.1s+1)`
- **Parameters**: loaded by model `PreLoadFcn` from `scripts/init_controller_params.m`

### Controller parameters

- $K_p \approx 8.27$, $K_i \approx 4.44$, $K_d \approx 3.81$, $T_f = 0.08$
- Setpoint weights: $b = 0.9$, $c = 0.05$
- Simulink filter coefficient: $N = 1/T_f$

## Quick start

1. Open MATLAB in this folder (or add it to the path).
2. Optional retune + export:

   ```matlab
   run scripts/design_controller.m
   ```

3. Open the model:

   ```matlab
   open_system('models/digital_stabilizing_controller.slx')
   ```

4. Full acceptance check (analysis + Simulink + plots):

   ```matlab
   run scripts/verify_requirements.m
   ```

## Files

```
design-digital-controller-simulink/
├── models/digital_stabilizing_controller.slx
├── scripts/
│   ├── design_controller.m       % current design + export
│   ├── init_controller_params.m  % model preload (auto-generated)
│   ├── verify_requirements.m     % full MIL gate + y/u plots
│   ├── explore_designs.m         % early PID / lead survey
│   ├── refine_design.m           % prefilter / PM comparison
│   ├── retune_settling.m         % search for 2 s ±2% envelope
│   └── soften_u.m                % search for lower peak |u|
├── docs/specs/algorithms/digitalStabilizingController/
├── controller_design.mat
└── README.md
```
