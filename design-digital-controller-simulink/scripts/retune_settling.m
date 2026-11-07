% Retune for: |y(t)-1| < 0.02 for all t >= 2 s, plus prior requirements
s = tf('s');
P = 1/(s^2 - 0.1*s + 1);
Ts = 0.1;
Pd = c2d(P, Ts, 'zoh');

reqs.tr_max = 1;
reqs.Mp_max = 10;
reqs.Sm_min = 0.5;
reqs.settle_band = 0.02;
reqs.settle_time = 2;

t = (0:Ts:20)';

best = [];
bestScore = Inf;
nPassPID = 0;
nPID = 0;

for pm = [45 50 55 58 60 62 65]
    opts = pidtuneOptions('PhaseMargin', pm, 'DesignFocus', 'reference-tracking');
    C = pidtune(Pd, 'PIDF', opts);
    L = C * Pd;
    S = feedback(1, L);
    for tau = [0.05 0.1 0.15 0.2 0.25 0.3 0.4 0.5]
        F = c2d(1/(tau*s + 1), Ts, 'tustin');
        T = feedback(L, 1) * F;
        name = sprintf('PM%d-tau%.2f', pm, tau);
        [pass, m] = evaluate_TF(name, T, S, t, reqs);
        nPID = nPID + 1;
        nPassPID = nPassPID + pass;
        m.C = C; m.F = F; m.pm = pm; m.tau = tau; m.pass = pass;
        if pass
            score = (m.ymax_after2 - m.ymin_after2) + 0.1*m.tr + 0.01*m.Mp;
            if score < bestScore
                bestScore = score;
                best = m;
            end
        end
    end
end

% Continuous PI+lead discretized + prefilter
for K = [6 8 10 12 15]
    for Td = [0.4 0.5 0.6 0.8]
        for a = [1.0 1.5 2.0 2.5 3.0]
            for N = [10 12 15]
                Cc = minreal(K * (1 + Td*s) / (1 + Td/N*s) * (s + a) / s);
                try
                    Cd = c2d(Cc, Ts, 'tustin');
                catch
                    continue
                end
                L = Cd * Pd;
                Tcl = feedback(L, 1);
                if ~isstable(Tcl), continue; end
                S = feedback(1, L);
                for tau = [0.05 0.1 0.15 0.2 0.3]
                    F = c2d(1/(tau*s + 1), Ts, 'tustin');
                    T = Tcl * F;
                    name = sprintf('Lead K=%d Td=%.1f a=%.1f N=%d tau=%.2f', K, Td, a, N, tau);
                    [pass, m] = evaluate_TF(name, T, S, t, reqs);
                    m.C = Cd; m.F = F; m.pm = NaN; m.tau = tau; m.pass = pass;
                    if pass
                        score = (m.ymax_after2 - m.ymin_after2) + 0.1*m.tr + 0.01*m.Mp;
                        if score < bestScore
                            bestScore = score;
                            best = m;
                        end
                    end
                end
            end
        end
    end
end

% 2-DOF PIDF2 sweep
for pm = [50 55 60 65]
    opts = pidtuneOptions('PhaseMargin', pm, 'DesignFocus', 'reference-tracking');
    C2 = pidtune(Pd, 'PIDF2', opts);
    Cyr = getPIDLoopResponse(C2, Pd, 'closed-loop');
    L = getPIDLoopResponse(C2, Pd, 'open-loop');
    S = feedback(1, L);
    name = sprintf('PIDF2-PM%d', pm);
    [pass, m] = evaluate_TF(name, Cyr, S, t, reqs);
    m.C = C2; m.F = tf(1, 1, Ts); m.pm = pm; m.tau = 0; m.pass = pass;
    if pass
        score = (m.ymax_after2 - m.ymin_after2) + 0.1*m.tr + 0.01*m.Mp;
        if score < bestScore
            bestScore = score;
            best = m;
        end
    end
end

fprintf('\n=== BEST ===\n');
if isempty(best)
    fprintf('No candidate met all requirements.\n');
else
    disp(best.name);
    fprintf('tr=%.3f Mp=%.2f Sm=%.3f tau=%.3f\n', best.tr, best.Mp, best.Sm, best.tau);
    if isa(best.C, 'pid')
        fprintf('Kp=%.6f Ki=%.6f Kd=%.6f Tf=%.6f\n', best.C.Kp, best.C.Ki, best.C.Kd, best.C.Tf);
    elseif isa(best.C, 'pid2')
        fprintf('PIDF2 Kp=%.6f Ki=%.6f Kd=%.6f Tf=%.6f b=%.4f c=%.4f\n', ...
            best.C.Kp, best.C.Ki, best.C.Kd, best.C.Tf, best.C.b, best.C.c);
    else
        [num, den] = tfdata(tf(best.C), 'v');
        fprintf('C_num=[%s]\n', sprintf('%.6g ', num));
        fprintf('C_den=[%s]\n', sprintf('%.6g ', den));
    end
end
fprintf('PIDF+prefilter passes: %d / %d\n', nPassPID, nPID);

function ok = in_band_after(y, t, band, t0, yfinal)
idx = t >= t0;
ok = all(abs(y(idx) - yfinal) <= band);
end

function [pass, m] = evaluate_TF(name, T, S, t, reqs)
m = struct();
m.name = name;
m.stable = isstable(T);
if ~m.stable
    fprintf('%s: UNSTABLE\n', name);
    pass = false;
    return
end
[y, ty] = step(T, t);
info = stepinfo(y, ty, 1);
m.tr = info.RiseTime;
m.Mp = info.Overshoot;
m.ts = info.SettlingTime;
m.Sm = 1/getPeakGain(S);
m.band2 = in_band_after(y, ty, reqs.settle_band, reqs.settle_time, 1);
m.y2 = interp1(ty, y, reqs.settle_time);
m.ymin_after2 = min(y(ty >= reqs.settle_time));
m.ymax_after2 = max(y(ty >= reqs.settle_time));
pass = m.stable && m.tr < reqs.tr_max && m.Mp < reqs.Mp_max && ...
    m.Sm > reqs.Sm_min && m.band2;
fprintf(['%s: tr=%.3f Mp=%.2f%% Sm=%.3f y(2)=%.3f ' ...
    'range[t>=2]=[%.3f,%.3f] band2=%d PASS=%d\n'], ...
    name, m.tr, m.Mp, m.Sm, m.y2, m.ymin_after2, m.ymax_after2, m.band2, pass);
end
