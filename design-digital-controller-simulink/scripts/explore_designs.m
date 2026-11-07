% Explore discrete controller designs for P = 1/(s^2-0.1*s+1)
s = tf('s');
P = 1/(s^2 - 0.1*s + 1);
Ts = 0.1;
Pd = c2d(P, Ts, 'zoh');

report = @(name, C, Plant) local_report(name, C, Plant);

% Continuous lead + PI families
Cpd = 8 * (1 + 0.6*s) / (1 + 0.06*s);
report('Cont lead', Cpd, P);

for a = [0.3 0.5 0.8 1.2 2]
    C = Cpd * (s + a) / s;
    report(sprintf('Cont PI+lead a=%.1f', a), C, P);
end

for K = [4 6 8 10 12 15]
    C = K * (1 + 0.6*s) / (1 + 0.06*s) * (s + 0.8) / s;
    report(sprintf('Cont K=%.0f a=0.8', K), C, P);
end

% Lead time-constant sweeps
for Td = [0.3 0.4 0.5 0.6 0.8]
    for N = [8 10 12]
        C = 10 * (1 + Td*s) / (1 + Td/N*s) * (s + 1) / s;
        report(sprintf('Cont Td=%.1f N=%d', Td, N), C, P);
    end
end

% Discretize promising continuous designs
candidates = {
    minreal(8*(1+0.6*s)/(1+0.06*s)*(s+0.8)/s)
    minreal(10*(1+0.5*s)/(1+0.05*s)*(s+1)/s)
    minreal(12*(1+0.4*s)/(1+0.04*s)*(s+1.2)/s)
    minreal(6*(1+0.7*s)/(1+0.07*s)*(s+0.6)/s)
    minreal(10*(1+0.6*s)/(1+0.06*s)*(s+1.5)/s)
    };

for i = 1:numel(candidates)
    Cc = candidates{i};
    for method = ["tustin", "matched", "zoh"]
        try
            Cd = c2d(Cc, Ts, method);
            report(sprintf('Cand%d-%s', i, method), Cd, Pd);
        catch ME
            fprintf('Cand%d-%s failed: %s\n', i, method, ME.message);
        end
    end
end

% Discrete pidtune
for pm = [50 55 60 65 70]
    opts = pidtuneOptions('PhaseMargin', pm, 'DesignFocus', 'reference-tracking');
    Cd = pidtune(Pd, 'PIDF', opts);
    report(sprintf('DiscPID PM%d', pm), Cd, Pd);
end

function local_report(name, C, Plant)
T = feedback(C * Plant, 1);
S = feedback(1, C * Plant);
if ~isstable(T)
    fprintf('%s: UNSTABLE\n', name);
    return
end
info = stepinfo(T);
Ms = getPeakGain(S);
fprintf('%s: tr=%.3f Mp=%.2f%% ts=%.2f Sm=%.3f dc=%.3f\n', ...
    name, info.RiseTime, info.Overshoot, info.SettlingTime, 1/Ms, dcgain(T));
end
