% Retune PIDF2 against hybrid Simulink closed-loop (source of truth)
projRoot = fileparts(fileparts(mfilename('fullpath')));
cd(projRoot);
model = 'digital_stabilizing_controller';
modelPath = fullfile(projRoot, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
load_system(modelPath);

s = tf('s');
P = 1/(s^2 - 0.1*s + 1);
Ts = 0.1;
Pd = c2d(P, Ts, 'zoh');
P_num = 1;
P_den = [1, -0.1, 1]; %#ok<NASGU>

best = [];
bestScore = Inf;

for pm = [40 45 48 50 52 55 58 60]
    opts = pidtuneOptions('PhaseMargin', pm, 'DesignFocus', 'reference-tracking');
    C = pidtune(Pd, 'PIDF2', opts);

    % Analysis metrics
    T = getPIDLoopResponse(C, Pd, 'closed-loop');
    L = getPIDLoopResponse(C, Pd, 'open-loop');
    S = feedback(1, L);
    Sm = 1/getPeakGain(S);
    if ~isstable(T) || Sm <= 0.5
        fprintf('PM=%d: skip (stable=%d Sm=%.3f)\n', pm, isstable(T), Sm);
        continue
    end

    % Push gains into base workspace for model
    Kp = C.Kp; Ki = C.Ki; Kd = C.Kd; Tf = C.Tf; b = C.b; c = C.c; %#ok<NASGU>
    assignin('base', 'Ts', Ts);
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ki', Ki);
    assignin('base', 'Kd', Kd);
    assignin('base', 'Tf', Tf);
    assignin('base', 'b', b);
    assignin('base', 'c', c);
    assignin('base', 'P_num', P_num);
    assignin('base', 'P_den', P_den);

    in = Simulink.SimulationInput(model);
    in = in.setModelParameter('StopTime', '20');
    in = in.setVariable('Ts', Ts);
    in = in.setVariable('Kp', Kp);
    in = in.setVariable('Ki', Ki);
    in = in.setVariable('Kd', Kd);
    in = in.setVariable('Tf', Tf);
    in = in.setVariable('b', b);
    in = in.setVariable('c', c);
    in = in.setVariable('P_num', P_num);
    in = in.setVariable('P_den', P_den);
    out = sim(in);
    y = out.get('y_sim');
    info = stepinfo(y.Data, y.Time, 1);
    inBand = all(abs(y.Data(y.Time >= 2) - 1) <= 0.02);
    yminA = min(y.Data(y.Time >= 2));
    ymaxA = max(y.Data(y.Time >= 2));
    pass = info.RiseTime < 1 && info.Overshoot < 10 && Sm > 0.5 && inBand;

    fprintf(['PM=%d: tr=%.3f Mp=%.2f%% Sm=%.3f range[t>=2]=[%.4f,%.4f] ' ...
        'band=%d PASS=%d\n'], pm, info.RiseTime, info.Overshoot, Sm, yminA, ymaxA, inBand, pass);

    if pass
        score = (ymaxA - yminA) + 0.05*info.Overshoot + 0.1*info.RiseTime;
        if score < bestScore
            bestScore = score;
            best = struct('pm', pm, 'C', C, 'Sm', Sm, 'info', info, ...
                'yminA', yminA, 'ymaxA', ymaxA);
        end
    end
end

% Also try manual setpoint-weight / gain tweaks around PM50
baseOpts = pidtuneOptions('PhaseMargin', 50, 'DesignFocus', 'reference-tracking');
C0 = pidtune(Pd, 'PIDF2', baseOpts);
for scaleKi = [1.0 1.25 1.5 1.75 2.0]
    for bb = [0.7 0.85 1.0]
        for cc = [0.2 0.4 0.6 0.8]
            C = C0;
            C.Ki = C0.Ki * scaleKi;
            C.b = bb;
            C.c = cc;
            T = getPIDLoopResponse(C, Pd, 'closed-loop');
            L = getPIDLoopResponse(C, Pd, 'open-loop');
            S = feedback(1, L);
            Sm = 1/getPeakGain(S);
            if ~isstable(T) || Sm <= 0.5
                continue
            end
            Kp = C.Kp; Ki = C.Ki; Kd = C.Kd; Tf = C.Tf; b = C.b; c = C.c;
            in = Simulink.SimulationInput(model);
            in = in.setModelParameter('StopTime', '20');
            in = in.setVariable('Ts', Ts);
            in = in.setVariable('Kp', Kp);
            in = in.setVariable('Ki', Ki);
            in = in.setVariable('Kd', Kd);
            in = in.setVariable('Tf', Tf);
            in = in.setVariable('b', b);
            in = in.setVariable('c', c);
            in = in.setVariable('P_num', P_num);
            in = in.setVariable('P_den', P_den);
            out = sim(in);
            y = out.get('y_sim');
            info = stepinfo(y.Data, y.Time, 1);
            inBand = all(abs(y.Data(y.Time >= 2) - 1) <= 0.02);
            yminA = min(y.Data(y.Time >= 2));
            ymaxA = max(y.Data(y.Time >= 2));
            pass = info.RiseTime < 1 && info.Overshoot < 10 && Sm > 0.5 && inBand;
            fprintf(['tweaks Ki*%.2f b=%.2f c=%.2f: tr=%.3f Mp=%.2f%% Sm=%.3f ' ...
                'range=[%.4f,%.4f] PASS=%d\n'], scaleKi, bb, cc, ...
                info.RiseTime, info.Overshoot, Sm, yminA, ymaxA, pass);
            if pass
                score = (ymaxA - yminA) + 0.05*info.Overshoot + 0.1*info.RiseTime;
                if score < bestScore
                    bestScore = score;
                    best = struct('pm', 50, 'C', C, 'Sm', Sm, 'info', info, ...
                        'yminA', yminA, 'ymaxA', ymaxA, ...
                        'note', sprintf('Ki*%.2f b=%.2f c=%.2f', scaleKi, bb, cc));
                end
            end
        end
    end
end

fprintf('\n=== BEST SIMULINK-PASSING DESIGN ===\n');
if isempty(best)
    disp('None found');
else
    disp(best);
    fprintf('Kp=%.6f Ki=%.6f Kd=%.6f Tf=%.6f b=%.6f c=%.6f\n', ...
        best.C.Kp, best.C.Ki, best.C.Kd, best.C.Tf, best.C.b, best.C.c);
end
