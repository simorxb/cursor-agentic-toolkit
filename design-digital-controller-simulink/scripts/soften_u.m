% Soften control effort while keeping all closed-loop requirements
s = tf('s');
P = 1 / (s^2 - 0.1*s + 1);
Ts = 0.1;
Pd = c2d(P, Ts, 'zoh');
tgrid = (0:Ts:20)';

best = [];
bestScore = Inf;

Cbase = pidtune(Pd, 'PIDF2', pidtuneOptions('PhaseMargin', 50, ...
    'DesignFocus', 'reference-tracking'));
[~, m0] = eval_design(Cbase, Pd, tgrid);
fprintf('BASELINE upk=%.2f udot=%.1f tr=%.2f Mp=%.1f Sm=%.2f band=%d\n', ...
    m0.upk, m0.udu, m0.tr, m0.Mp, m0.Sm, m0.band);

for pm = [40 45 50 55 60]
    warning('off', 'Control:design:pidtune1');
    opts = pidtuneOptions('PhaseMargin', pm, 'DesignFocus', 'reference-tracking');
    C0 = pidtune(Pd, 'PIDF2', opts);
    for c = [0 0.1 0.2 0.3]
        for b = [0.6 0.8 1.0]
            for kdScale = [0.3 0.5 0.65 0.8 1.0]
                for kpScale = [0.7 0.85 1.0]
                    C = C0;
                    C.b = b;
                    C.c = c;
                    C.Kd = C0.Kd * kdScale;
                    C.Kp = C0.Kp * kpScale;
                    % Slightly heavier derivative filter helps kick
                    C.Tf = max(C0.Tf, 0.08);
                    [pass, m] = eval_design(C, Pd, tgrid);
                    if ~pass
                        continue
                    end
                    score = m.upk + 0.01 * m.udu;
                    fprintf(['PASS PM=%d b=%.1f c=%.1f kd*%.2f kp*%.2f | ' ...
                        'tr=%.2f Mp=%.1f Sm=%.2f upk=%.1f udot=%.0f\n'], ...
                        pm, b, c, kdScale, kpScale, m.tr, m.Mp, m.Sm, m.upk, m.udu);
                    if score < bestScore
                        bestScore = score;
                        best = struct('C', C, 'm', m, 'pm', pm, ...
                            'kdScale', kdScale, 'kpScale', kpScale);
                    end
                end
            end
        end
    end
end

% 2DOF + mild reference prefilter on r before PID
for pm = [45 50 55]
    C0 = pidtune(Pd, 'PIDF2', pidtuneOptions('PhaseMargin', pm, ...
        'DesignFocus', 'reference-tracking'));
    C0.c = 0; % no D kick from reference
    C0.b = 1;
    for tau = [0.1 0.2 0.3]
        F = c2d(1/(tau*s+1), Ts, 'tustin');
        % Approximate analysis: replace r with F*r into 2DOF PID
        % Build via getPIDLoopResponse then cascade F on input
        T0 = getPIDLoopResponse(C0, Pd, 'closed-loop');
        Tu0 = getPIDLoopResponse(C0, Pd, 'controller-effort');
        T = T0 * F;
        Tu = Tu0 * F;
        L = getPIDLoopResponse(C0, Pd, 'open-loop');
        S = feedback(1, L);
        if ~isstable(T), continue; end
        [y, ~] = step(T, tgrid);
        [u, ~] = step(Tu, tgrid);
        info = stepinfo(y, tgrid, 1);
        Sm = 1/getPeakGain(S);
        band = all(abs(y(tgrid >= 2) - 1) <= 0.02);
        pass = info.RiseTime < 1 && info.Overshoot < 10 && Sm > 0.5 && band;
        upk = max(abs(u));
        udu = max(abs(diff(u))) / Ts;
        if pass
            score = upk + 0.01 * udu;
            fprintf(['PASS 2DOF+F PM=%d tau=%.2f | tr=%.2f Mp=%.1f Sm=%.2f ' ...
                'upk=%.1f udot=%.0f\n'], pm, tau, info.RiseTime, ...
                info.Overshoot, Sm, upk, udu);
            if score < bestScore
                bestScore = score;
                best = struct('kind', 'pid2_prefilter', 'C', C0, 'F', F, ...
                    'm', struct('tr', info.RiseTime, 'Mp', info.Overshoot, ...
                    'Sm', Sm, 'upk', upk, 'udu', udu, 'band', band), ...
                    'pm', pm, 'tau', tau);
            end
        end
    end
end

fprintf('\n=== BEST ===\n');
if isempty(best)
    disp('No candidate');
else
    disp(best);
    C = best.C;
    fprintf('Kp=%.4f Ki=%.4f Kd=%.4f Tf=%.4f b=%.4f c=%.4f upk=%.2f\n', ...
        C.Kp, C.Ki, C.Kd, C.Tf, C.b, C.c, best.m.upk);
end

function [pass, m] = eval_design(C, Pd, tgrid)
T = getPIDLoopResponse(C, Pd, 'closed-loop');
Tu = getPIDLoopResponse(C, Pd, 'controller-effort');
L = getPIDLoopResponse(C, Pd, 'open-loop');
S = feedback(1, L);
m = struct('stable', isstable(T), 'tr', Inf, 'Mp', Inf, 'Sm', 0, ...
    'band', false, 'upk', Inf, 'udu', Inf);
if ~m.stable
    pass = false;
    return
end
[y, ~] = step(T, tgrid);
[u, ~] = step(Tu, tgrid);
info = stepinfo(y, tgrid, 1);
m.tr = info.RiseTime;
m.Mp = info.Overshoot;
m.Sm = 1 / getPeakGain(S);
m.band = all(abs(y(tgrid >= 2) - 1) <= 0.02);
m.upk = max(abs(u));
m.udu = max(abs(diff(u))) / Pd.Ts;
pass = m.tr < 1 && m.Mp < 10 && m.Sm > 0.5 && m.band;
end
