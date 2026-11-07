# digitalStabilizingController — System + Architecture (Quick Spec)

**Status:** Approved for implementation  
**Last Updated:** 2026-11-07  
**Sample time:** Ts = 0.1 s  

---

## 1. Executive Summary

Discrete 2-DOF PIDF controller that stabilizes the unstable plant  
`P(s) = 1/(s^2 - 0.1*s + 1)` and tracks a unit step with specified transient, settling-envelope, and robustness margins. Setpoint weights `(b, c)` shape the reference channel without changing the feedback loop margins.

---

## 2. Goals & Success Metrics

| Goal | Metric | Target | Method |
|------|--------|--------|--------|
| G1 Stability | Closed-loop poles | All stable | Eigenvalues / `isstable` |
| G2 Robustness | Sensitivity margin Sm = 1/‖S‖∞ | > 0.5 | Frequency-domain `getPeakGain(S)` |
| G3 Speed | Rise time (10–90%) | < 1 s | Step response |
| G4 Damping | Overshoot | < 10% | Step response |
| G5 Settling envelope | \|y(t) − 1\| | ≤ 0.02 for all t ≥ 2 s | Step response |
| G6 Digital | Controller sample time | 0.1 s | Discrete 2-DOF PID / ZOH to plant |

---

## 3. External Interface

| Signal | Dir | Unit | Description |
|--------|-----|------|-------------|
| `r` | in | 1 | Reference (step command) |
| `y` | in | 1 | Plant output measurement |
| `u` | out | 1 | Control effort to plant (ZOH held) |

**Parameters (calibratable):** `Kp`, `Ki`, `Kd`, `Tf`, `b`, `c`, `Ts`  
**Fixed:** plant coefficients `P_num`, `P_den`

**Polarity:** 2-DOF form  
`u = Kp*(b*r − y) + Ki*∫(r − y) + Kd*N*d/dt_f(c*r − y)` (discrete Forward-Euler I/D).

---

## 4. Operating Scenarios

| ID | Scenario | Expected behavior |
|----|----------|-------------------|
| S1 | Unit step from rest | Stable tracking; meets G2–G5 |
| S2 | Startup (zero IC) | No algebraic loop; bounded `u`, `y` |

**Non-goals (v1):** actuator saturation / anti-windup, disturbance observers, multi-rate scheduling, code generation constraints beyond discrete PID.

---

## 5. Architecture

```
r -----> [Discrete PIDF 2DOF] -----> [Plant P(s)] -----> y
              ^                                         |
              +-----------------------------------------+
```

| Component | Role | Notes |
|-----------|------|-------|
| Discrete PIDF 2DOF | Stabilizing + command shaping | Parallel PIDF, Forward Euler; weights `b`,`c` |
| Plant | Unstable process | Continuous TF `1/(s^2-0.1s+1)` |

**Algebraic loops:** none — discrete controller breaks the loop.  
**Anti-windup:** not required in v1 (unsaturated demo).

---

## 6. Design Basis (verified in MATLAB)

- Discrete PIDF2 from `pidtune(Pd,'PIDF2')`, PhaseMargin = 50°, then detuned (`Kp×0.75`, `Kd×0.65`, `Tf=0.08`, `b=0.9`, `c=0.05`) for smoother `u`.  
- Low `c` limits derivative action on the reference (main source of the control kick).  
- Achieved (analysis/Simulink): tr ≈ 0.59 s, Mp ≈ 8.1%, Sm ≈ 0.67, \|y−1\| ≤ 2% for t ≥ 2 s, peak \|u\| ≈ 10.

---

## 7. Implementation + Test Plan (combined)

### Build order
1. `scripts/design_controller.m` → export gains  
2. `scripts/init_controller_params.m` preload  
3. Simulink model `models/digital_stabilizing_controller.slx`  
4. `scripts/verify_requirements.m` MIL checks  

### Tests
| Test | Stimulus | Pass |
|------|----------|------|
| T1 Stability | Unit step, 20 s | Bounded y,u; no divergence |
| T2 Rise time | Unit step | tr < 1 s |
| T3 Overshoot | Unit step | Mp < 10% |
| T4 Settling envelope | Unit step | \|y−1\| ≤ 0.02 for t ≥ 2 s |
| T5 Sensitivity margin | From open-loop L | Sm > 0.5 |
