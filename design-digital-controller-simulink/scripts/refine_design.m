% Refine discrete controller candidates against requirements
s = tf('s');
P = 1/(s^2 - 0.1*s + 1);
Ts = 0.1;
Pd = c2d(P, Ts, 'zoh');

reqs = struct('tr_max', 1, 'Mp_max', 10, 'Sm_min', 0.5);

% --- Candidate A: discrete PIDF, PM = 65 ---
opts = pidtuneOptions('PhaseMargin', 65, 'DesignFocus', 'reference-tracking');
CdA = pidtune(Pd, 'PIDF', opts);
[passA, metricsA] = check_reqs('A-PIDF-PM65', CdA, Pd, reqs);

% --- Candidate B: discrete PIDF2 (2-DOF) ---
CdB = pidtune(Pd, 'PIDF2', opts);
Cyr = getPIDLoopResponse(CdB, Pd, 'closed-loop');
% Sensitivity S = 1/(1+L); use feedback path of 2DOF PID
L = getPIDLoopResponse(CdB, Pd, 'open-loop');
Sresp = feedback(1, L);
infoB = stepinfo(Cyr);
MsB = getPeakGain(Sresp);
SmB = 1/MsB;
fprintf('B-PIDF2-PM65: tr=%.3f Mp=%.2f%% ts=%.2f Sm=%.3f stable=%d\n', ...
    infoB.RiseTime, infoB.Overshoot, infoB.SettlingTime, SmB, isstable(Cyr));
passB = isstable(Cyr) && infoB.RiseTime < reqs.tr_max && ...
    infoB.Overshoot < reqs.Mp_max && SmB > reqs.Sm_min; %#ok<NASGU>

% --- Candidate C: PM60 with first-order prefilter ---
opts60 = pidtuneOptions('PhaseMargin', 60, 'DesignFocus', 'reference-tracking');
CdCfb = pidtune(Pd, 'PIDF', opts60);
for tau = [0.2 0.3 0.4 0.5 0.6]
    F = c2d(1/(tau*s+1), Ts, 'tustin');
    % 2DOF: u = Cd*(F*r - y)  => T = Cd*Pd*F / (1+Cd*Pd), S unchanged
    L = CdCfb * Pd;
    T = feedback(L, 1) * F;
    S = feedback(1, L);
    info = stepinfo(T);
    Sm = 1/getPeakGain(S);
    fprintf('C-PM60-prefilter tau=%.1f: tr=%.3f Mp=%.2f%% ts=%.2f Sm=%.3f\n', ...
        tau, info.RiseTime, info.Overshoot, info.SettlingTime, Sm);
end

% --- Candidate D: discrete LQR + integral via place / dlqr on augmented plant ---
[A, B, C, D] = ssdata(ss(Pd));
n = size(A, 1);
Aa = [A, zeros(n, 1); -C, 1];
Ba = [B; 0];
Qa = diag([10, 10, 5]);
Ra = 1;
Ka = dlqr(Aa, Ba, Qa, Ra);
K = Ka(1:n);
Ki = Ka(end);
fprintf('D-LQR gains: K=[%g %g], Ki=%g\n', K(1), K(2), Ki);

% Build controller TF from integral state-feedback form for SISO analysis:
% For unit reference, xi+ = xi + (r-y), u = -K x - Ki xi
% Equivalent compensator needs observer - use pole placement observer
% Simpler: use PI state-feedback analysis via ss feedback later in Simulink.
% Here evaluate with full-state (optimistic) for reference only:
sys_cl_fs = ss(Aa - Ba*Ka, [zeros(n,1); 1], [C, 0], 0, Ts);
infoD = stepinfo(sys_cl_fs);
fprintf('D-LQR full-state (optimistic): tr=%.3f Mp=%.2f%% ts=%.2f\n', ...
    infoD.RiseTime, infoD.Overshoot, infoD.SettlingTime);

% --- Sweep PM finely around 63-67 ---
fprintf('\nFine PM sweep:\n');
best = struct('pm', NaN, 'score', Inf, 'Cd', []);
for pm = 62:0.5:68
    opts = pidtuneOptions('PhaseMargin', pm, 'DesignFocus', 'reference-tracking');
    Cd = pidtune(Pd, 'PIDF', opts);
    T = feedback(Cd*Pd, 1);
    S = feedback(1, Cd*Pd);
    if ~isstable(T), continue; end
    info = stepinfo(T);
    Sm = 1/getPeakGain(S);
    ok = info.RiseTime < 1 && info.Overshoot < 10 && Sm > 0.5;
    score = info.SettlingTime + 2*info.Overshoot; % prefer faster settle, low Mp
    fprintf('  PM=%.1f: tr=%.3f Mp=%.2f%% ts=%.2f Sm=%.3f ok=%d\n', ...
        pm, info.RiseTime, info.Overshoot, info.SettlingTime, Sm, ok);
    if ok && score < best.score
        best.pm = pm;
        best.score = score;
        best.Cd = Cd;
        best.info = info;
        best.Sm = Sm;
    end
end

fprintf('\nSelected best PIDF: PM=%.1f ts=%.2f Mp=%.2f Sm=%.3f\n', ...
    best.pm, best.info.SettlingTime, best.info.Overshoot, best.Sm);
disp(best.Cd);

% Export numerator/denominator
[num, den] = tfdata(tf(best.Cd), 'v');
fprintf('num = [%s]\n', sprintf('%.12g ', num));
fprintf('den = [%s]\n', sprintf('%.12g ', den));

% Save design artifacts
Kp = best.Cd.Kp; Ki = best.Cd.Ki; Kd = best.Cd.Kd; Tf = best.Cd.Tf; %#ok<NASGU>
C_num = num; C_den = den; %#ok<NASGU>
designPM = best.pm; %#ok<NASGU>
save(fullfile(fileparts(mfilename('fullpath')), '..', 'controller_design.mat'), ...
    'Kp', 'Ki', 'Kd', 'Tf', 'Ts', 'C_num', 'C_den', 'designPM', 'Pd', 'P');

function [pass, metrics] = check_reqs(name, C, Plant, reqs)
T = feedback(C * Plant, 1);
S = feedback(1, C * Plant);
metrics = struct();
metrics.stable = isstable(T);
if metrics.stable
    info = stepinfo(T);
    metrics.tr = info.RiseTime;
    metrics.Mp = info.Overshoot;
    metrics.ts = info.SettlingTime;
    metrics.Sm = 1/getPeakGain(S);
else
    metrics.tr = Inf; metrics.Mp = Inf; metrics.ts = Inf; metrics.Sm = 0;
end
pass = metrics.stable && metrics.tr < reqs.tr_max && ...
    metrics.Mp < reqs.Mp_max && metrics.Sm > reqs.Sm_min;
fprintf('%s: tr=%.3f Mp=%.2f%% ts=%.2f Sm=%.3f PASS=%d\n', ...
    name, metrics.tr, metrics.Mp, metrics.ts, metrics.Sm, pass);
end
