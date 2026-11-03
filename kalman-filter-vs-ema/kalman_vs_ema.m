%% Kalman filter vs exponential moving average
% Scalar random walk, F = 1 and H = 1:
%   x(k+1) = x(k) + w(k),  w ~ N(0, Q)
%   y(k)   = x(k) + v(k),  v ~ N(0, R)
%
% Predict does not move the estimate. The update is therefore
%   xhat = xhat + K (y - xhat) = (1 - K) xhat + K y
% Once K has settled at Kinf, that is an exponential moving average
% with alpha = Kinf. The steady predicted variance is the positive root of
%   P^2 - Q P - Q R = 0:
%   Pminus = (Q + sqrt(Q^2 + 4 Q R)) / 2
%   Kinf   = Pminus / (Pminus + R)
%
% Two places this stops being true:
%   start-up, where P is still large and K is near 1
%   a ramp, which the random-walk model cannot follow
%
% Seed is fixed. Do not rerun until a plot looks nicer.

%% Parameters
Q = 0.01;
R = 1;
Ts = 1;
nSteps = 2000;
P0 = 10;
x0True = 5;
x0Hat = 0;
rampVelocity = 0.05;
velocityNoiseIntensity = 1e-6;
seed = 1;

k = (0:nSteps - 1)';

%% Steady-state gain, three ways
% Closed form above.
PminusInf = (Q + sqrt(Q^2 + 4 * Q * R)) / 2;
Kinf = PminusInf / (PminusInf + R);
alpha = Kinf;

% Same Riccati recursion the filter runs, started from P0, until K stops moving.
P = P0;
Kriccati = NaN;
for riccatiIterations = 1:10000
    Pminus = P + Q;
    Knew = Pminus / (Pminus + R);
    Pnew = (1 - Knew) * Pminus;
    if riccatiIterations > 1 && abs(Knew - Kriccati) < 1e-15
        Kriccati = Knew;
        P = Pnew;
        break
    end
    Kriccati = Knew;
    P = Pnew;
end
PminusRiccati = P + Q;

% Control Riccati with A = F' and B = H'. Here F = H = 1, so idare(1, 1, Q, R)
% returns the predicted variance, and its gain output is Kinf.
[PminusIdare, KinfIdare] = idare(1, 1, Q, R);
KinfIdare = KinfIdare(1);

gainMismatch = max(abs([Kinf - Kriccati, Kinf - KinfIdare, ...
    PminusInf - PminusRiccati, PminusInf - PminusIdare]));
fprintf('Kinf closed form: %.12f\n', Kinf);
fprintf('Kinf Riccati:     %.12f after %d iterations\n', Kriccati, riccatiIterations);
fprintf('Kinf idare:       %.12f\n', KinfIdare);
fprintf('Pminus closed:    %.12f\n', PminusInf);
fprintf('Pminus Riccati:   %.12f\n', PminusRiccati);
fprintf('Pminus idare:     %.12f\n', PminusIdare);
if gainMismatch > 1e-10
    error('Steady-state gain checks disagree by %.3e.', gainMismatch);
end

%% Case 1: random walk
% Both filters see one noise draw. rng is reset again for the ramp so each
% case reproduces on its own from seed 1.
rng(seed, 'twister');
xTrue = zeros(nSteps, 1);
xTrue(1) = x0True;
processNoise = sqrt(Q) * randn(nSteps - 1, 1);
for stepIndex = 1:nSteps - 1
    xTrue(stepIndex + 1) = xTrue(stepIndex) + processNoise(stepIndex);
end
y = xTrue + sqrt(R) * randn(nSteps, 1);

[xhatKf, K] = randomWalkKalman(y, Q, R, P0, x0Hat);
xhatEma = exponentialMovingAverage(y, alpha, x0Hat);

% Constant-gain update written as a Kalman correction, not as the EMA line.
% For F = 1 these are the same recursion, so the estimates must agree.
xhatConstant = zeros(nSteps, 1);
xhat = x0Hat;
for stepIndex = 1:nSteps
    xhat = xhat + Kinf * (y(stepIndex) - xhat);
    xhatConstant(stepIndex) = xhat;
end
constantGainMismatch = max(abs(xhatConstant - xhatEma));

% K is near Kinf by about step 50. The state error built up before that
% then decays by (1 - alpha) each step, so "after the transient" depends
% on the window. Report several windows; do not pick one to match a target.
gapAfter50 = max(abs(xhatKf(51:end) - xhatEma(51:end)));
gapAfter200 = max(abs(xhatKf(201:end) - xhatEma(201:end)));
gapAfter1000 = max(abs(xhatKf(1001:end) - xhatEma(1001:end)));
settleIndex = find(abs(K - Kinf) < 1e-3, 1, 'first');
kSettled = settleIndex - 1;

fprintf('\nRandom walk, seed %d\n', seed);
fprintf('K at k = 0:        %.6f\n', K(1));
fprintf('|K - Kinf| < 1e-3 from k = %d onward\n', kSettled);
fprintf('max |KF - EMA|, k >= 50:   %.3e\n', gapAfter50);
fprintf('max |KF - EMA|, k >= 200:  %.3e\n', gapAfter200);
fprintf('max |KF - EMA|, k >= 1000: %.3e\n', gapAfter1000);
fprintf('max |constant-gain KF - EMA|: %.3e\n', constantGainMismatch);
fprintf('1/alpha = %.2f steps\n', 1 / alpha);

%% Case 2: ramp
% Truth is a constant velocity. The 1-state filter still assumes a random
% walk, so once its gain has settled it is the EMA, and it lags by the
% same amount. The lag is the missing velocity state.
rng(seed, 'twister');
xRamp = rampVelocity * k;
yRamp = xRamp + sqrt(R) * randn(nSteps, 1);

xhatEmaRamp = exponentialMovingAverage(yRamp, alpha, x0Hat);
[xhatKfRamp, ~] = randomWalkKalman(yRamp, Q, R, P0, x0Hat);
% Velocity prior is 0. The ramp velocity is rampVelocity. P0 is large, so
% the first few updates are allowed to learn that velocity.
[xhatCvRamp, cvFinalGain] = constantVelocityKalman( ...
    yRamp, Ts, velocityNoiseIntensity, R, P0, [0; 0]);

% Positive lag means the estimate is behind the truth.
steady = k >= 1000;
theoreticalLag = rampVelocity * (1 - alpha) / alpha;
emaLag = mean(xRamp(steady) - xhatEmaRamp(steady));
kfLag = mean(xRamp(steady) - xhatKfRamp(steady));
cvMeanError = mean(xhatCvRamp(steady) - xRamp(steady));
rampFilterGap = max(abs(xhatKfRamp(steady) - xhatEmaRamp(steady)));

fprintf('\nRamp, velocity %.3f, seed %d, mean over k >= 1000\n', rampVelocity, seed);
fprintf('theoretical EMA lag:  %.6f\n', theoreticalLag);
fprintf('simulated EMA lag:    %.6f\n', emaLag);
fprintf('simulated 1-state lag: %.6f\n', kfLag);
fprintf('2-state mean error (estimate - truth): %.6f\n', cvMeanError);
fprintf('max |1-state - EMA| on that window: %.3e\n', rampFilterGap);
fprintf('2-state final gain [position; velocity]: %.6f, %.6f\n', ...
    cvFinalGain(1), cvFinalGain(2));

% Both filters have unity DC gain, so they follow a stretch of
% positive measurement noise. The apparent lag is the formula minus
% the mean of that noise.
noiseMean = mean(yRamp(steady) - xRamp(steady));
fprintf('mean measurement noise, k >= 1000: %.6f\n', noiseMean);
fprintf('theory - noise mean:               %.6f\n', theoreticalLag - noiseMean);
fprintf('EMA lag minus (theory - noise mean): %.6f\n', ...
    emaLag - (theoreticalLag - noiseMean));
fprintf('2-state error minus noise mean:    %.6f\n', cvMeanError - noiseMean);

absGap = abs(xhatKf - xhatEma);
kGapBelow1e9 = NaN;
for stepIndex = 1:nSteps
    if max(absGap(stepIndex:end)) < 1e-9
        kGapBelow1e9 = stepIndex - 1;
        break
    end
end
fprintf('max |KF - EMA| stays below 1e-9 from k = %d\n', kGapBelow1e9);

%% Case 3: Kinf is a function of Q/R only
qrRatio = [1e-4, 1e-3, 1e-2, 1e-1, 1];
KinfTable = zeros(size(qrRatio));
fprintf('\nQ/R        Kinf\n');
for ratioIndex = 1:numel(qrRatio)
    KinfTable(ratioIndex) = steadyStateGain(qrRatio(ratioIndex), 1);
    fprintf('%.0e   %.6f\n', qrRatio(ratioIndex), KinfTable(ratioIndex));
end

%% Figures
figDir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end
oldFigures = findall(0, 'Type', 'figure', 'Tag', 'kalmanVsEma');
delete(oldFigures);

colorMeas = [0.82 0.82 0.82];
colorTruth = [0.15 0.15 0.15];
colorKf = [0.00 0.35 0.70];
colorEma = [0.85 0.33 0.10];
colorCv = [0.00 0.50 0.35];

startup = k <= 100;

figStartup = portraitFigure();
layoutStartup = tiledlayout(figStartup, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layoutStartup, {'After start-up,', 'the Kalman filter is an EMA'}, ...
    'FontName', 'Arial', 'FontSize', 28, 'FontWeight', 'bold');

axStartup = nexttile(layoutStartup);
plotRandomWalk(k(startup), y(startup), xTrue(startup), xhatKf(startup), ...
    xhatEma(startup), colorMeas, colorTruth, colorKf, colorEma);
styleAxes(axStartup, 'First 100 steps', 'step k', 'state', '');

axFull = nexttile(layoutStartup);
plotRandomWalk(k, y, xTrue, xhatKf, xhatEma, ...
    colorMeas, colorTruth, colorKf, colorEma);
styleAxes(axFull, 'All 2000 steps', 'step k', 'state', '');

% One legend for both panels, outside the axes, so it does not cover the traces.
sharedLegend = legend(axStartup);
sharedLegend.Layout.Tile = 'south';
sharedLegend.FontName = 'Arial';
sharedLegend.FontSize = 18;
sharedLegend.Box = 'off';

figGain = portraitFigure();
layoutGain = tiledlayout(figGain, 1, 1, 'Padding', 'compact');
nexttile(layoutGain);
plot(k(startup), K(startup), 'Color', colorKf, 'LineWidth', 2.5, ...
    'DisplayName', 'Kalman gain');
hold on
yline(Kinf, '--', 'Color', 'k', 'LineWidth', 2, ...
    'DisplayName', sprintf('K_\\infty = %.4f', Kinf));
hold off
xlim([0 100]);
ylim([0 1]);
styleAxes(gca, {'The gain settles', 'at the EMA weight'}, ...
    'step k, first 100 of 2000', 'Kalman gain', 'northeast');

figRamp = portraitFigure();
layoutRamp = tiledlayout(figRamp, 1, 1, 'Padding', 'compact');
nexttile(layoutRamp);
plot(k, xhatEmaRamp - xRamp, 'Color', colorEma, 'LineWidth', 1.6, ...
    'DisplayName', 'EMA');
hold on
plot(k, xhatKfRamp - xRamp, '--', 'Color', colorKf, 'LineWidth', 1.6, ...
    'DisplayName', '1-state Kalman filter');
plot(k, xhatCvRamp - xRamp, 'Color', colorCv, 'LineWidth', 2.2, ...
    'DisplayName', '2-state Kalman filter');
yline(0, 'Color', [0.55 0.55 0.55], 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(-theoreticalLag, '--', 'Color', 'k', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('theoretical lag, %.3f', -theoreticalLag));
hold off
styleAxes(gca, {'Random-walk model lags a ramp', 'Velocity model tracks it'}, ...
    'step k', 'estimate - truth', 'northeast');

figAlpha = portraitFigure();
layoutAlpha = tiledlayout(figAlpha, 1, 1, 'Padding', 'compact');
nexttile(layoutAlpha);
qrFine = logspace(-4, 0, 250);
KinfFine = zeros(size(qrFine));
for ratioIndex = 1:numel(qrFine)
    KinfFine(ratioIndex) = steadyStateGain(qrFine(ratioIndex), 1);
end
semilogx(qrFine, KinfFine, 'Color', colorKf, 'LineWidth', 2.5, ...
    'DisplayName', 'K_\infty');
hold on
semilogx(qrRatio, KinfTable, 'o', 'Color', colorEma, ...
    'MarkerFaceColor', colorEma, 'MarkerSize', 12, 'LineWidth', 1.5, ...
    'DisplayName', 'example ratios');
hold off
styleAxes(gca, 'Q/R picks the EMA weight', 'Q/R', 'K_\infty = \alpha', 'northwest');

startupFile = fullfile(figDir, 'random_walk_kf_and_ema.png');
gainFile = fullfile(figDir, 'kalman_gain.png');
rampFile = fullfile(figDir, 'ramp_tracking_error.png');
alphaFile = fullfile(figDir, 'alpha_from_qr.png');
savePortrait(figStartup, startupFile);
savePortrait(figGain, gainFile);
savePortrait(figRamp, rampFile);
savePortrait(figAlpha, alphaFile);

%% Stop if a deterministic check fails, or a noisy one is not even close
if abs(K(1) - 0.91) > 0.02
    error('Gain at k = 0 is %.4f, expected about 0.91.', K(1));
end
if kSettled > 80 || ~all(abs(K(settleIndex:end) - Kinf) < 1e-3)
    error('Gain has not settled near Kinf by k = %d.', kSettled);
end
if constantGainMismatch > 1e-8
    error('Constant-gain filter and EMA differ by %.3e.', constantGainMismatch);
end
if abs(emaLag - theoreticalLag) > 0.1 || abs(kfLag - theoreticalLag) > 0.1
    error('Ramp lag (EMA %.3f, 1-state %.3f) is far from theory %.3f.', ...
        emaLag, kfLag, theoreticalLag);
end
if abs(cvMeanError) > 0.15
    error('2-state mean ramp error is %.3f, not near zero.', cvMeanError);
end
if abs(emaLag - (theoreticalLag - noiseMean)) > 0.02
    error('EMA lag %.3f is not theory minus the noise mean.', emaLag);
end
if abs(cvMeanError - noiseMean) > 0.03
    error('2-state mean error %.3f does not follow the noise mean %.3f.', ...
        cvMeanError, noiseMean);
end
if rampFilterGap > 1e-6
    error('On the ramp, 1-state KF and EMA still differ by %.3e after k = 1000.', ...
        rampFilterGap);
end

fprintf('\nWrote\n  %s\n  %s\n  %s\n  %s\n', ...
    startupFile, gainFile, rampFile, alphaFile);

%% Local functions
function [xhat, K] = randomWalkKalman(y, Q, R, P0, x0Hat)
% Scalar random-walk filter. Predict adds Q and leaves the mean still.
% A large P produces a large gain, so the first updates follow y.
    n = numel(y);
    xhat = zeros(n, 1);
    K = zeros(n, 1);
    x = x0Hat;
    P = P0;
    for k = 1:n
        Pminus = P + Q;
        K(k) = Pminus / (Pminus + R);
        x = x + K(k) * (y(k) - x);
        P = (1 - K(k)) * Pminus;
        xhat(k) = x;
    end
end

function xhat = exponentialMovingAverage(y, alpha, x0Hat)
    n = numel(y);
    xhat = zeros(n, 1);
    x = x0Hat;
    for k = 1:n
        x = (1 - alpha) * x + alpha * y(k);
        xhat(k) = x;
    end
end

function [xhat, finalGain] = constantVelocityKalman(y, Ts, q, R, P0, x0Hat)
% Constant-velocity model. Position is measured, velocity is estimated.
% Process noise is q times the discrete white-noise acceleration covariance,
% so a tiny q means "the velocity is almost constant."
    n = numel(y);
    F = [1, Ts; 0, 1];
    H = [1, 0];
    Qcv = q * [Ts^3 / 3, Ts^2 / 2; Ts^2 / 2, Ts];
    xhat = zeros(n, 1);
    x = x0Hat(:);
    P = P0 * eye(2);
    K = zeros(2, 1);
    for k = 1:n
        x = F * x;
        P = F * P * F' + Qcv;
        S = H * P * H' + R;
        K = (P * H') / S;
        x = x + K * (y(k) - H * x);
        P = (eye(2) - K * H) * P;
        xhat(k) = x(1);
    end
    finalGain = K;
end

function Kinf = steadyStateGain(Q, R)
    Pminus = (Q + sqrt(Q^2 + 4 * Q * R)) / 2;
    Kinf = Pminus / (Pminus + R);
end

function fig = portraitFigure()
% 1080 by 1350 px at 150 dpi. 7.2 in by 9 in.
    fig = figure('Units', 'inches', 'Position', [0.5 0.5 7.2 9], ...
        'Color', 'w', 'Tag', 'kalmanVsEma', 'WindowStyle', 'normal', ...
        'PaperUnits', 'inches', 'PaperSize', [7.2 9], ...
        'PaperPosition', [0 0 7.2 9]);
end

function styleAxes(ax, titleText, xLabel, yLabel, legendLocation)
    ax.FontName = 'Arial';
    ax.FontSize = 20;
    ax.LineWidth = 1.1;
    ax.Box = 'on';
    ax.TickDir = 'out';
    grid(ax, 'on');
    ax.GridAlpha = 0.18;
    xlabel(ax, xLabel, 'FontSize', 24);
    ylabel(ax, yLabel, 'FontSize', 24);
    title(ax, titleText, 'FontName', 'Arial', 'FontSize', 26, 'FontWeight', 'bold');
    if strlength(string(legendLocation)) > 0
        legend(ax, 'FontSize', 18, 'Box', 'off', 'Location', legendLocation);
    end
end

function plotRandomWalk(k, y, xTrue, xhatKf, xhatEma, colorMeas, colorTruth, colorKf, colorEma)
    plot(k, y, 'Color', colorMeas, 'LineWidth', 1.1, 'DisplayName', 'measurements');
    hold on
    plot(k, xhatKf, 'Color', colorKf, 'LineWidth', 2.4, 'DisplayName', 'Kalman filter');
    plot(k, xhatEma, '--', 'Color', colorEma, 'LineWidth', 2.0, 'DisplayName', 'EMA');
    plot(k, xTrue, 'Color', colorTruth, 'LineWidth', 1.6, 'DisplayName', 'truth');
    hold off
end

function savePortrait(fig, filePath)
% print uses PaperPosition, set on the figure to 7.2 in by 9 in.
% exportgraphics on this machine wrote 1080 x 1324.
    drawnow;
    print(fig, filePath, '-dpng', '-r150');
    info = imfinfo(filePath);
    if info.Width ~= 1080 || info.Height ~= 1350
        error('Saved %s as %d x %d, not 1080 x 1350.', filePath, info.Width, info.Height);
    end
end
