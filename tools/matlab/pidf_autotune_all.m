%% PIDF Auto-Tune - All Motors
% Live Script source.
% Open this file in MATLAB Live Editor and Save As .mlx if desired.
%
% Pipeline for each motor:
%   latest multistep CSV
%   -> estimate 1P0Z / 1P1Z / 2P0Z / 2P1Z
%   -> choose highest validation fit
%   -> pidtune(...,'PIDF')
%   -> scale controller output from % duty to STM32 PWM counts
%   -> export models/controllers/results to workspace and CSV

clear;
clc;
close all;

motors = ["WR","WL","BR","BL","CV"];

Ts = 0.01;                 % STM32 control period: 100 Hz
estimateFraction = 0.70;   % 70% estimation, 30% validation
pwmARR = 4999;             % TIM PWM ARR
dutyPercentFullScale = 100;
firmwareScale = pwmARR / dutyPercentFullScale;   % 49.99 counts per % duty

results = table();

%% Process all motors

for m = 1:numel(motors)

    motor = motors(m);

    %% Find latest multistep file

    files = dir(fullfile(pwd,'**',motor + "_multistep_*.csv"));

    if isempty(files)
        warning("No multistep CSV found for %s. Skipping.",motor);
        continue;
    end

    [~,idxLatest] = max([files.datenum]);
    file = fullfile(files(idxLatest).folder,files(idxLatest).name);

    fprintf("\n========================================\n");
    fprintf("%s MOTOR\n",motor);
    fprintf("Using: %s\n",file);
    fprintf("========================================\n");

    %% Read and reconstruct uniform 100 Hz data

    T = readtable(file);

    tick = double(T.control_tick);
    dutyPercent = double(T.command_duty) * 100;
    rpm = double(T.rpm);

    [tick,ia] = unique(tick,'stable');
    dutyPercent = dutyPercent(ia);
    rpm = rpm(ia);

    completeTick = (tick(1):tick(end))';

    dutyUniform = interp1(tick,dutyPercent,completeTick,'previous');
    rpmUniform = interp1(tick,rpm,completeTick,'linear');

    valid = isfinite(dutyUniform) & isfinite(rpmUniform);
    dutyUniform = dutyUniform(valid);
    rpmUniform = rpmUniform(valid);

    data = iddata( ...
        rpmUniform, ...
        dutyUniform, ...
        Ts, ...
        'InputName','Duty Cycle', ...
        'InputUnit','%', ...
        'OutputName','Motor Speed', ...
        'OutputUnit','RPM');

    %% Estimation / validation split

    N = size(data.OutputData,1);
    splitIndex = floor(estimateFraction*N);

    if splitIndex < 20 || (N-splitIndex) < 20
        warning("%s does not contain enough data. Skipping.",motor);
        continue;
    end

    dataEst = data(1:splitIndex);
    dataVal = data(splitIndex+1:end);

    %% Estimate candidate transfer functions

    modelNames = ["1P0Z","1P1Z","2P0Z","2P1Z"];
    poleCount = [1 1 2 2];
    zeroCount = [0 1 0 1];

    models = cell(4,1);
    fits = -inf(4,1);

    for k = 1:4

        try
            models{k} = tfest(dataEst,poleCount(k),zeroCount(k));

            plantCandidate = tf(models{k});

            % A motor-speed plant should be stable.
            if ~isstable(plantCandidate)
                fprintf("%s: unstable estimate, excluded\n",modelNames(k));
                continue;
            end

            [~,fit] = compare(dataVal,models{k});
            fits(k) = fit(1);

            fprintf("%s fit: %.2f %%\n",modelNames(k),fits(k));

        catch ME
            fprintf("%s failed: %s\n",modelNames(k),ME.message);
        end

    end

    %% Choose best identified plant

    [bestFit,bestIndex] = max(fits);

    if ~isfinite(bestFit)
        warning("No valid model found for %s.",motor);
        continue;
    end

    bestModelName = modelNames(bestIndex);
    bestIDModel = models{bestIndex};
    bestPlant = tf(bestIDModel);

    fprintf("\nBest model: %s, fit = %.2f %%\n",bestModelName,bestFit);
    disp(bestPlant);

    %% PIDF automatic tuning

    C_percent = pidtune(bestPlant,'PIDF');

    % Plant input is % duty.
    % Firmware controller output is PWM counts (0..4999), therefore:
    %   K_firmware = K_percent * 4999/100
    Kp_STM32 = C_percent.Kp * firmwareScale;
    Ki_STM32 = C_percent.Ki * firmwareScale;
    Kd_STM32 = C_percent.Kd * firmwareScale;
    Tf_STM32 = C_percent.Tf;

    C_STM32 = pid( ...
        Kp_STM32, ...
        Ki_STM32, ...
        Kd_STM32, ...
        Tf_STM32);

    fprintf("\nPIDF from MATLAB, output in %% duty:\n");
    fprintf("Kp = %.9g\n",C_percent.Kp);
    fprintf("Ki = %.9g\n",C_percent.Ki);
    fprintf("Kd = %.9g\n",C_percent.Kd);
    fprintf("Tf = %.9g s\n",C_percent.Tf);

    fprintf("\nPIDF for STM32, output in PWM counts:\n");
    fprintf("Kp = %.9g\n",Kp_STM32);
    fprintf("Ki = %.9g\n",Ki_STM32);
    fprintf("Kd = %.9g\n",Kd_STM32);
    fprintf("Tf = %.9g s\n",Tf_STM32);

    if Kp_STM32 < 0 || Ki_STM32 < 0 || Tf_STM32 < 0
        warning("%s produced invalid Kp/Ki/Tf signs. Do not send these gains.",motor);
    end

    if Kd_STM32 < 0
        warning("%s PIDF has negative Kd (%.6g). This is allowed, but verify the closed-loop response before hardware testing.",motor,Kd_STM32);
    end

    if ~isstable(feedback(C_percent*bestPlant,1))
        warning("%s tuned closed loop is unstable. Do not send these gains to hardware.",motor);
    end

    %% Export to base workspace

    assignin('base',motor + "_best_tf",bestPlant);
    assignin('base',motor + "_PIDF_percent",C_percent);
    assignin('base',motor + "_PIDF_STM32",C_STM32);

    %% Closed-loop response - one figure per motor

    closedLoop = feedback(C_percent*bestPlant,1);

    figure('Name',motor + " PIDF Auto-Tune");
    step(closedLoop);
    grid on;
    title(motor + " Motor - Auto-Tuned PIDF Closed-Loop Response");

    %% Store result

    row = table( ...
        motor, ...
        bestModelName, ...
        bestFit, ...
        C_percent.Kp, ...
        C_percent.Ki, ...
        C_percent.Kd, ...
        C_percent.Tf, ...
        Kp_STM32, ...
        Ki_STM32, ...
        Kd_STM32, ...
        Tf_STM32, ...
        'VariableNames',{ ...
        'Motor', ...
        'BestModel', ...
        'BestFit_pct', ...
        'Kp_percent', ...
        'Ki_percent', ...
        'Kd_percent', ...
        'Tf_s', ...
        'Kp_STM32', ...
        'Ki_STM32', ...
        'Kd_STM32', ...
        'Tf_STM32_s'});

    results = [results; row];

end

%% Final PIDF table

disp(" ");
disp("========================================");
disp("PIDF AUTO-TUNE RESULTS");
disp("========================================");
disp(results);

assignin('base','PIDF_results',results);

%% Save for the Raspberry Pi updater

outputCSV = fullfile(pwd,'pidf_autotune_results.csv');
writetable(results,outputCSV);

fprintf("\nSaved PIDF parameters to:\n%s\n",outputCSV);
fprintf("\nThe *_PIDF_STM32 variables are scaled for the current firmware PWM range (ARR = %d).\n",pwmARR);
