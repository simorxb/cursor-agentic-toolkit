% VERIFY_REQUIREMENTS MIL checks for digital stabilizing controller

close all
clc

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projRoot, 'scripts'));
cd(projRoot);

run(fullfile(projRoot, 'scripts', 'design_controller.m'));
projRoot = fileparts(fileparts(mfilename('fullpath')));
cd(projRoot);
load(fullfile(projRoot, 'controller_design.mat'), 'C', 'Pd', 'metrics', 'Ts');

%% Frequency-domain + settling-band (analysis)
T = getPIDLoopResponse(C, Pd, 'closed-loop');
Tu = getPIDLoopResponse(C, Pd, 'controller-effort');
L = getPIDLoopResponse(C, Pd, 'open-loop');
S = feedback(1, L);
Sm = 1 / getPeakGain(S);
assert(isstable(T), 'FAIL: closed-loop unstable (analysis)');
assert(Sm > 0.5, 'FAIL: sensitivity margin Sm=%.3f (req > 0.5)', Sm);

tgrid = (0:Ts:20)';
[ya, ta] = step(T, tgrid);
[ua, ~] = step(Tu, tgrid);
assert(all(abs(ya(ta >= 2) - 1) <= 0.02), 'FAIL: analysis settling band');

%% Simulink MIL step test
model = 'digital_stabilizing_controller';
modelPath = fullfile(projRoot, 'models', [model '.slx']);
assert(isfile(modelPath), 'Model not found: %s', modelPath);

if bdIsLoaded(model)
    close_system(model, 0);
end
load_system(modelPath);
run(fullfile(projRoot, 'scripts', 'init_controller_params.m'));

in = Simulink.SimulationInput(model);
in = in.setModelParameter('StopTime', '20');
out = sim(in);

y = out.get('y_sim');
u = out.get('u_sim');
info = stepinfo(y.Data, y.Time, 1);
inBand = all(abs(y.Data(y.Time >= 2) - 1) <= 0.02);
yminAfter = min(y.Data(y.Time >= 2));
ymaxAfter = max(y.Data(y.Time >= 2));

fprintf('\n=== Requirement verification ===\n');
fprintf('Analysis Sm:       %.3f  (req > 0.5)     %s\n', Sm, passfail(Sm > 0.5));
fprintf('Sim rise time:     %.3f s (req < 1)      %s\n', info.RiseTime, passfail(info.RiseTime < 1));
fprintf('Sim overshoot:     %.2f %% (req < 10)    %s\n', info.Overshoot, passfail(info.Overshoot < 10));
fprintf('Sim +/-2%% t>=2s:  [%0.4f,%0.4f]         %s\n', yminAfter, ymaxAfter, passfail(inBand));
fprintf('Sim settle:        %.2f s\n', info.SettlingTime);
fprintf('Sim final value:   %.4f\n', y.Data(end));

assert(info.RiseTime < 1, 'FAIL: rise time');
assert(info.Overshoot < 10, 'FAIL: overshoot');
assert(inBand, 'FAIL: settling band');

fprintf('\nAll requirements PASS.\n');

figure('Name', 'Step response verification');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(ta, ya, 'LineWidth', 2); hold on;
plot(y.Time, y.Data, '--', 'LineWidth', 2);
yline(1, ':');
yline(1.1, '--', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(1.02, '--', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(0.98, '--', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(2, ':', 'HandleVisibility', 'off');
grid on;
ylabel('y');
legend('Analysis (discrete)', 'Simulink', 'Location', 'best');
title(sprintf('Unit step | tr=%.2fs Mp=%.1f%% Sm=%.2f | band@2s OK', ...
    info.RiseTime, info.Overshoot, Sm));

nexttile;
plot(ta, ua, 'LineWidth', 2); hold on;
plot(u.Time, u.Data, '--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('u');
legend('Analysis (discrete)', 'Simulink', 'Location', 'best');
title('Control effort');

function tf = passfail(cond)
if cond
    tf = 'PASS';
else
    tf = 'FAIL';
end
end
