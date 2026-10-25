%% Controller for an unstable plant
% Plant:
%   P(s) = 1 / (s^2 - 0.1 s + 1)
%
% Requirements:
%   - stable closed loop
%   - sensitivity margin > 0.5
%   - 10% to 90% rise time < 1 s
%   - overshoot < 10%
%
% The sensitivity margin is the distance from the Nyquist plot of the loop
% transfer L to the critical point -1:
%   margin = min |1 + L(jw)| = 1 / ||S||_inf,   S = 1 / (1 + L)

%% Plant
s = tf('s');
plant = 1 / (s^2 - 0.1 * s + 1);

plantPoles = pole(plant);
fprintf('Open-loop poles: %.4f %+.4fi and %.4f %+.4fi\n', ...
    real(plantPoles(1)), imag(plantPoles(1)), real(plantPoles(2)), imag(plantPoles(2)));

%% Design parameters
% The poles lie near magnitude 1, so the loop has to cross over above that
% frequency to stabilise the plant. A dominant pair at wn = 3.2 rad/s with
% damping 0.7 is fast enough for a rise time below 1 s and damped enough for
% the overshoot limit. The third pole keeps the integrator mode from slowing
% the response. The derivative filter sits about a decade above wn.
dampingRatio = 0.7;
naturalFrequency = 3.2;
realPole = -8;
derivativeTimeConstant = 0.02;

minimumSensitivityMargin = 0.5;
maximumRiseTime = 1;
maximumOvershoot = 10;

%% Pole placement on the plant plus an integrator
% States are [y; ydot; xi], with xi_dot = r - y.
plantA = [0 1; -1 0.1];
plantB = [0; 1];
plantC = [1 0];
augmentedA = [plantA, zeros(2, 1); -plantC, 0];
augmentedB = [plantB; 0];

dampedFrequency = naturalFrequency * sqrt(1 - dampingRatio^2);
desiredPoles = [-dampingRatio * naturalFrequency + 1i * dampedFrequency, ...
    -dampingRatio * naturalFrequency - 1i * dampedFrequency, ...
    realPole];

stateFeedback = place(augmentedA, augmentedB, desiredPoles);
proportionalGain = stateFeedback(1);
derivativeGain = stateFeedback(2);
% xi_dot = -y in the regulator model, so the integral gain changes sign.
integralGain = -stateFeedback(3);

%% Two-degree-of-freedom PID
% Proportional and derivative terms act on y, not on the reference, so a step
% command is not differentiated. The same three terms are in the feedback path,
% which is what sets the sensitivity.
%   u = (Ki / s) (r - y) - Kp y - (Kd s / (tau s + 1)) y
derivativeFilter = derivativeGain * s / (derivativeTimeConstant * s + 1);
feedbackController = proportionalGain + integralGain / s + derivativeFilter;

plant.InputName = 'u';
plant.OutputName = 'y';

integralPath = tf(integralGain, [1 0]);
integralPath.InputName = 'e';
integralPath.OutputName = 'ui';

measurementFeedback = proportionalGain + derivativeFilter;
measurementFeedback.InputName = 'y';
measurementFeedback.OutputName = 'uy';

errorJunction = sumblk('e = r - y');
actuationJunction = sumblk('u = ui - uy');

closedLoop = connect(plant, integralPath, measurementFeedback, ...
    errorJunction, actuationJunction, 'r', {'y', 'u'});
outputResponse = closedLoop('y', 'r');
controlResponse = closedLoop('u', 'r');

loopTransfer = feedbackController * (1 / (s^2 - 0.1 * s + 1));
sensitivity = feedback(1, loopTransfer);

%% Check the requirements
sensitivityMargin = 1 / hinfnorm(sensitivity);
responseInfo = stepinfo(outputResponse);
classicalMargins = allmargin(loopTransfer);
timeHorizon = 4;
[controlStep, controlTime] = step(controlResponse, timeHorizon);
peakControl = max(abs(controlStep));

stable = isstable(outputResponse);
fprintf('\nGains: Kp = %.3f, Ki = %.3f, Kd = %.3f, tau = %.3f s\n', ...
    proportionalGain, integralGain, derivativeGain, derivativeTimeConstant);
fprintf('Closed-loop stable: %s\n', string(stable));
fprintf('Sensitivity margin: %.3f (required > %.1f)\n', ...
    sensitivityMargin, minimumSensitivityMargin);
fprintf('Rise time:          %.3f s (required < %.0f s)\n', ...
    responseInfo.RiseTime, maximumRiseTime);
fprintf('Overshoot:          %.2f %% (required < %.0f %%)\n', ...
    responseInfo.Overshoot, maximumOvershoot);
fprintf('Settling time (2%%): %.3f s\n', responseInfo.SettlingTime);
fprintf('Phase margin:       %.1f deg at %.2f rad/s\n', ...
    classicalMargins.PhaseMargin, classicalMargins.PMFrequency);
fprintf('Minimum stable gain factor: %.3f\n', min(classicalMargins.GainMargin));
fprintf('Peak |u| for a unit step: %.2f\n', peakControl);

assert(stable, 'The closed loop is not stable.');
assert(sensitivityMargin > minimumSensitivityMargin, ...
    'Sensitivity margin is %.3f, which is not above %.1f.', ...
    sensitivityMargin, minimumSensitivityMargin);
assert(responseInfo.RiseTime < maximumRiseTime, ...
    'Rise time is %.3f s, which is not below %.0f s.', ...
    responseInfo.RiseTime, maximumRiseTime);
assert(responseInfo.Overshoot < maximumOvershoot, ...
    'Overshoot is %.2f %%, which is not below %.0f %%.', ...
    responseInfo.Overshoot, maximumOvershoot);

%% Step response
% The one-degree-of-freedom PID puts the same gains on the tracking error,
% so the derivative term acts on the reference step.
[outputStep, outputTime] = step(outputResponse, timeHorizon);
oneDofOutput = feedback(loopTransfer, 1);
[oneDofStep, oneDofTime] = step(oneDofOutput, timeHorizon);
oneDofControl = minreal(feedbackController / (1 + loopTransfer));
[oneDofControlStep, oneDofControlTime] = step(oneDofControl, timeHorizon);

figure
tiledlayout(2, 1)

nexttile
plot(outputTime, outputStep, 'LineWidth', 1.2)
hold on
plot(oneDofTime, oneDofStep, 'LineWidth', 1.2)
yline(1 + maximumOvershoot / 100, '--', '10% overshoot', 'HandleVisibility', 'off')
hold off
xlabel('Time (s)')
ylabel('y')
title(sprintf('Output  |  2-DOF rise %.2f s, overshoot %.1f%%', ...
    responseInfo.RiseTime, responseInfo.Overshoot))
legend('2-DOF PID', '1-DOF PID', 'Location', 'southeast')
grid on

nexttile
plot(controlTime, controlStep, 'LineWidth', 1.2)
hold on
plot(oneDofControlTime, oneDofControlStep, 'LineWidth', 1.2)
hold off
xlabel('Time (s)')
ylabel('u')
title('Control effort')
legend('2-DOF PID', '1-DOF PID', 'Location', 'northeast')
grid on

%% Sensitivity
frequency = logspace(-1, 2, 400);
sensitivityMagnitude = squeeze(abs(freqresp(sensitivity, frequency)));

figure
semilogx(frequency, sensitivityMagnitude, 'LineWidth', 1.2)
hold on
yline(1 / minimumSensitivityMargin, '--', '||S|| limit for margin 0.5')
hold off
xlabel('Frequency (rad/s)')
ylabel('|S|')
title(sprintf('Sensitivity  |  margin %.2f', sensitivityMargin))
grid on
xlim([frequency(1), frequency(end)])
