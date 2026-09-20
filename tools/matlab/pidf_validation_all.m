%% PIDF Step Validation - All Motors
% Open this source in MATLAB Live Editor and save as .mlx.
%
% For each motor this script:
%   1. Finds the latest *_pid_validation_*.csv
%   2. Plots reference RPM and measured RPM in its own figure
%   3. Calculates simple step-response validation metrics

clear;
clc;
close all;

motors = ["WR","WL","BR","BL","CV"];

% Light filtering for readable validation plots.
% Set to 0 to plot completely raw RPM.
averageWindow_s = 0.05;

results = table();

%% Process latest validation file for each motor

for m = 1:numel(motors)

    motor = motors(m);

    files = dir(fullfile(pwd,'**',motor + "_pid_validation_*.csv"));

    if isempty(files)
        warning("No PID validation CSV found for %s.",motor);
        continue;
    end

    [~,idxLatest] = max([files.datenum]);
    file = fullfile(files(idxLatest).folder,files(idxLatest).name);

    fprintf("\n========================================\n");
    fprintf("%s PIDF VALIDATION\n",motor);
    fprintf("Using: %s\n",file);
    fprintf("========================================\n");

    T = readtable(file);

    time = double(T.host_time_s);
    reference = double(T.reference_rpm);
    measuredRaw = double(T.measured_rpm);

    valid = isfinite(time) & isfinite(reference) & isfinite(measuredRaw);
    time = time(valid);
    reference = reference(valid);
    measuredRaw = measuredRaw(valid);

    if numel(time) < 10
        warning("%s validation file has too few samples.",motor);
        continue;
    end

    % Normalize time to start at zero.
    time = time - time(1);

    dt = median(diff(time));

    if averageWindow_s > 0
        windowSamples = max(1,round(averageWindow_s/dt));
        measured = movmean(measuredRaw,windowSamples);
    else
        windowSamples = 1;
        measured = measuredRaw;
    end

    %% Find active step interval

    nonzero = abs(reference) > 1e-9;

    if ~any(nonzero)
        warning("%s file contains no nonzero RPM step.",motor);
        continue;
    end

    iStart = find(nonzero,1,'first');
    iEnd = find(nonzero,1,'last');

    referenceRPM = median(reference(iStart:iEnd));

    tStep = time(iStart:iEnd) - time(iStart);
    yStep = measured(iStart:iEnd);

    % Estimate achieved steady-state RPM from final 20% of step interval.
    nStep = numel(yStep);
    iSteady = max(1,floor(0.80*nStep));

    steadyRPM = mean(yStep(iSteady:end));
    steadyStateError = referenceRPM - steadyRPM;

    if abs(referenceRPM) > 1e-9
        steadyStateError_pct = ...
            100*abs(steadyStateError)/abs(referenceRPM);
    else
        steadyStateError_pct = NaN;
    end

    % Make positive-reference equivalent for stepinfo so reverse steps work too.
    direction = sign(referenceRPM);
    if direction == 0
        direction = 1;
    end

    yForMetrics = direction*yStep;
    finalForMetrics = direction*steadyRPM;

    try
        S = stepinfo( ...
            yForMetrics, ...
            tStep, ...
            finalForMetrics, ...
            'SettlingTimeThreshold',0.02);

        riseTime = S.RiseTime;
        settlingTime = S.SettlingTime;
        overshoot = S.Overshoot;

    catch
        riseTime = NaN;
        settlingTime = NaN;
        overshoot = NaN;
    end

    %% Plot - one figure per motor

    figure('Name',motor + " PIDF Step Validation");

    plot(time,reference,'--','LineWidth',1.3);
    hold on;
    plot(time,measured,'LineWidth',1.2);

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('Motor Speed (RPM)');
    title(motor + " Motor - PIDF Step Validation");

    legend('Reference RPM','Measured RPM','Location','best');

    hold off;

    %% Store metrics

    kp = T.kp(1);
    ki = T.ki(1);
    kd = T.kd(1);
    tf = T.tf_s(1);

    row = table( ...
        motor, ...
        referenceRPM, ...
        steadyRPM, ...
        steadyStateError_pct, ...
        riseTime, ...
        settlingTime, ...
        overshoot, ...
        kp,ki,kd,tf, ...
        'VariableNames',{ ...
        'Motor', ...
        'Reference_RPM', ...
        'SteadyState_RPM', ...
        'SteadyStateError_pct', ...
        'RiseTime_s', ...
        'SettlingTime_s', ...
        'Overshoot_pct', ...
        'Kp', ...
        'Ki', ...
        'Kd', ...
        'Tf_s'});

    results = [results; row];

    fprintf("Reference       : %.2f RPM\n",referenceRPM);
    fprintf("Steady-state    : %.2f RPM\n",steadyRPM);
    fprintf("Steady error    : %.2f %%\n",steadyStateError_pct);
    fprintf("Rise time       : %.3f s\n",riseTime);
    fprintf("Settling time   : %.3f s\n",settlingTime);
    fprintf("Overshoot       : %.2f %%\n",overshoot);

end

%% Validation summary

disp(" ");
disp("========================================");
disp("PIDF VALIDATION RESULTS");
disp("========================================");
disp(results);

assignin('base','PIDF_validation_results',results);
