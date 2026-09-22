%% PIDF Auto-Tune - Best Continuous/Discrete Model
% Live Script source.
%
% Pipeline for each motor:
%   latest multistep CSV
%   -> estimate:
%        Continuous 1P0Z
%        Continuous 2P1Z
%        Discrete   1P0Z
%        Discrete   2P1Z
%   -> choose highest validation fit
%   -> if best plant is continuous, discretize plant using ZOH at Ts = 0.01 s
%   -> tune a DISCRETE PIDF using trapezoidal integral + derivative formulas
%      to match the STM32 PIDF implementation
%   -> scale controller output from % duty to STM32 PWM counts
%   -> export models/controllers/results to workspace and CSV
%
% STM32 implementation:
%   Integral:
%       I[k] = I[k-1] + Ts/2 * (e[k] + e[k-1])
%
%   Filtered derivative:
%       D[k] = ad*D[k-1] + bd*(e[k] - e[k-1])
%       ad = (2*Tf - Ts)/(2*Tf + Ts)
%       bd = 2*Kd/(2*Tf + Ts)
%
% These are the trapezoidal / Tustin discrete formulas.

clear;
clc;
close all;

motors = ["WR","WL","BR","BL","CV"];

%% Settings

Ts = 0.01;                 % STM32 control period = 100 Hz
estimateFraction = 0.70;   % 70% estimation / 30% validation

pwmARR = 4999;
dutyPercentFullScale = 100;

% Convert MATLAB controller output [% duty] to STM32 PWM counts.
firmwareScale = pwmARR / dutyPercentFullScale;   % 49.99

results = table();

%% Process all motors

for m = 1:numel(motors)

    motor = motors(m);

    %% Find latest multistep file

    files = dir(fullfile( ...
        pwd, ...
        '**', ...
        motor + "_multistep_*.csv"));

    if isempty(files)
        warning("No multistep CSV found for %s. Skipping.",motor);
        continue;
    end

    [~,idxLatest] = max([files.datenum]);

    file = fullfile( ...
        files(idxLatest).folder, ...
        files(idxLatest).name);

    fprintf("\n========================================\n");
    fprintf("%s MOTOR\n",motor);
    fprintf("Using: %s\n",file);
    fprintf("========================================\n");

    %% Read data

    T = readtable(file);

    tick = double(T.control_tick);
    dutyPercent = double(T.command_duty) * 100;
    rpm = double(T.rpm);

    %% Remove duplicate control ticks

    [tick,ia] = unique(tick,'stable');

    dutyPercent = dutyPercent(ia);
    rpm = rpm(ia);

    %% Reconstruct uniform 100 Hz data

    completeTick = (tick(1):tick(end))';

    dutyUniform = interp1( ...
        tick, ...
        dutyPercent, ...
        completeTick, ...
        'previous');

    rpmUniform = interp1( ...
        tick, ...
        rpm, ...
        completeTick, ...
        'linear');

    %% Remove invalid values

    valid = ...
        isfinite(dutyUniform) & ...
        isfinite(rpmUniform);

    dutyUniform = dutyUniform(valid);
    rpmUniform = rpmUniform(valid);

    %% Create identification data

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

    %% ============================================================
    %  IDENTIFY CONTINUOUS + DISCRETE CANDIDATE MODELS
    % =============================================================

    modelNames = [
        "C_1P0Z"
        "C_2P1Z"
        "D_1P0Z"
        "D_2P1Z"
    ];

    modelLongNames = [
        "Continuous 1P0Z"
        "Continuous 2P1Z"
        "Discrete 1P0Z"
        "Discrete 2P1Z"
    ];

    models = cell(4,1);
    fits = -inf(4,1);

    %% Continuous 1P0Z

    try

        models{1} = tfest( ...
            dataEst, ...
            1, ...
            0, ...
            'Ts', ...
            0);

        G = tf(models{1});

        if isstable(G)

            [~,fit] = compare(dataVal,models{1});

            fits(1) = fit(1);

        else

            fprintf("Continuous 1P0Z: unstable estimate, excluded.\n");

        end

    catch ME

        fprintf("Continuous 1P0Z failed: %s\n",ME.message);

    end

    %% Continuous 2P1Z

    try

        models{2} = tfest( ...
            dataEst, ...
            2, ...
            1, ...
            'Ts', ...
            0);

        G = tf(models{2});

        if isstable(G)

            [~,fit] = compare(dataVal,models{2});

            fits(2) = fit(1);

        else

            fprintf("Continuous 2P1Z: unstable estimate, excluded.\n");

        end

    catch ME

        fprintf("Continuous 2P1Z failed: %s\n",ME.message);

    end

    %% Discrete 1P0Z

    try

        models{3} = tfest( ...
            dataEst, ...
            1, ...
            0, ...
            'Ts', ...
            Ts);

        G = tf(models{3});

        if isstable(G)

            [~,fit] = compare(dataVal,models{3});

            fits(3) = fit(1);

        else

            fprintf("Discrete 1P0Z: unstable estimate, excluded.\n");

        end

    catch ME

        fprintf("Discrete 1P0Z failed: %s\n",ME.message);

    end

    %% Discrete 2P1Z

    try

        models{4} = tfest( ...
            dataEst, ...
            2, ...
            1, ...
            'Ts', ...
            Ts);

        G = tf(models{4});

        if isstable(G)

            [~,fit] = compare(dataVal,models{4});

            fits(4) = fit(1);

        else

            fprintf("Discrete 2P1Z: unstable estimate, excluded.\n");

        end

    catch ME

        fprintf("Discrete 2P1Z failed: %s\n",ME.message);

    end

    %% Print validation fits

    fprintf("\nMODEL VALIDATION FIT\n");
    fprintf("----------------------------------------\n");

    for k = 1:4

        if isfinite(fits(k))

            fprintf( ...
                "%-20s = %7.2f %%\n", ...
                modelLongNames(k), ...
                fits(k));

        else

            fprintf( ...
                "%-20s = excluded\n", ...
                modelLongNames(k));

        end

    end

    fprintf("----------------------------------------\n");

    %% Choose best identified plant

    [bestFit,bestIndex] = max(fits);

    if ~isfinite(bestFit)

        warning("No valid model found for %s.",motor);

        continue;

    end

    bestModelName = modelNames(bestIndex);
    bestModelLongName = modelLongNames(bestIndex);

    bestIDModel = models{bestIndex};
    bestPlant = tf(bestIDModel);

    fprintf( ...
        "\nBEST MODEL: %s = %.2f %%\n", ...
        bestModelLongName, ...
        bestFit);

    disp(bestPlant);

    %% ============================================================
    %  PREPARE PLANT FOR THE ACTUAL 100 Hz DIGITAL CONTROLLER
    % =============================================================

    if bestPlant.Ts == 0

        % The physical PWM command is held between controller updates.
        % Therefore use ZOH to represent the continuous plant at 100 Hz.
        plantForTune = c2d( ...
            bestPlant, ...
            Ts, ...
            'zoh');

        tunePlantSource = "Continuous model -> ZOH at 100 Hz";

    else

        % Discrete identification already represents the sampled plant.
        if abs(bestPlant.Ts - Ts) > 1e-12

            error( ...
                "%s best discrete plant Ts = %.9g s, expected %.9g s.", ...
                motor, ...
                bestPlant.Ts, ...
                Ts);

        end

        plantForTune = bestPlant;

        tunePlantSource = "Direct discrete identified model";

    end

    %% ============================================================
    %  PIDF TEMPLATE MATCHING STM32 IMPLEMENTATION
    % =============================================================

    % Current STM32 PIDF uses:
    %
    % Integral:
    %   trapezoidal integration
    %
    % Derivative filter:
    %   Tustin / trapezoidal discretization
    %
    % Supplying Ctemplate to pidtune forces MATLAB to tune the same
    % discrete controller form instead of using its default Forward Euler.

    Ctemplate = pid( ...
        1, ...                       % Kp placeholder
        1, ...                       % Ki placeholder
        1, ...                       % Kd placeholder
        0.01, ...                    % Tf placeholder
        Ts, ...
        'IFormula','Trapezoidal', ...
        'DFormula','Trapezoidal');

    %% Tune PIDF

    [C_percent,tuneInfo] = pidtune( ...
        plantForTune, ...
        Ctemplate);

    %% Confirm controller implementation

    fprintf("\nPIDF TUNING IMPLEMENTATION\n");
    fprintf("----------------------------------------\n");
    fprintf("Controller sample time = %.6f s\n",C_percent.Ts);
    fprintf("Integral formula        = %s\n",C_percent.IFormula);
    fprintf("Derivative formula      = %s\n",C_percent.DFormula);
    fprintf("Tuning plant            = %s\n",tunePlantSource);

    if isfield(tuneInfo,'CrossoverFrequency')

        fprintf( ...
            "Crossover frequency     = %.4f rad/s\n", ...
            tuneInfo.CrossoverFrequency);

    end

    if isfield(tuneInfo,'PhaseMargin')

        fprintf( ...
            "Phase margin            = %.2f deg\n", ...
            tuneInfo.PhaseMargin);

    end

    %% MATLAB PIDF gains
    %
    % Input to plant  = duty %
    % Output of plant = RPM
    %
    % Therefore controller output from MATLAB is duty %.

    Kp_percent = C_percent.Kp;
    Ki_percent = C_percent.Ki;
    Kd_percent = C_percent.Kd;
    Tf_percent = C_percent.Tf;

    %% Convert gains to STM32 PWM-count output units

    Kp_STM32 = Kp_percent * firmwareScale;
    Ki_STM32 = Ki_percent * firmwareScale;
    Kd_STM32 = Kd_percent * firmwareScale;

    % Tf is a time constant, so it is NOT scaled.
    Tf_STM32 = Tf_percent;

    %% Create equivalent STM32 controller model in MATLAB

    C_STM32 = pid( ...
        Kp_STM32, ...
        Ki_STM32, ...
        Kd_STM32, ...
        Tf_STM32, ...
        Ts, ...
        'IFormula','Trapezoidal', ...
        'DFormula','Trapezoidal');

    %% Print gains

    fprintf("\nPIDF FROM MATLAB - output in %% duty\n");
    fprintf("----------------------------------------\n");
    fprintf("Kp = %.9g\n",Kp_percent);
    fprintf("Ki = %.9g\n",Ki_percent);
    fprintf("Kd = %.9g\n",Kd_percent);
    fprintf("Tf = %.9g s\n",Tf_percent);

    fprintf("\nPIDF FOR STM32 - output in PWM counts\n");
    fprintf("----------------------------------------\n");
    fprintf("Kp = %.9g\n",Kp_STM32);
    fprintf("Ki = %.9g\n",Ki_STM32);
    fprintf("Kd = %.9g\n",Kd_STM32);
    fprintf("Tf = %.9g s\n",Tf_STM32);

    %% Firmware parameter validity check

    firmwareValid = ...
        isfinite(Kp_STM32) && ...
        isfinite(Ki_STM32) && ...
        isfinite(Kd_STM32) && ...
        isfinite(Tf_STM32) && ...
        Kp_STM32 >= 0 && ...
        Ki_STM32 >= 0 && ...
        Tf_STM32 >= 0 && ...
        Kp_STM32 <= 100000 && ...
        Ki_STM32 <= 100000 && ...
        abs(Kd_STM32) <= 100000 && ...
        Tf_STM32 <= 10;

    if ~firmwareValid

        warning( ...
            "%s tuned PIDF violates current STM32 PID_SET limits.", ...
            motor);

    end

    if Kd_STM32 < 0

        fprintf( ...
            "NOTE: %s has negative Kd. Current STM32 firmware allows signed Kd.\n", ...
            motor);

    end

    %% Closed-loop model using actual digital controller form

    closedLoop = feedback( ...
        C_percent * plantForTune, ...
        1);

    closedLoopStable = isstable(closedLoop);

    if ~closedLoopStable

        warning( ...
            "%s tuned discrete closed loop is unstable.", ...
            motor);

    end

    %% Export models/controllers to base workspace

    % Identified candidates
    if ~isempty(models{1})
        assignin('base',motor + "_Gc_1P0Z",models{1});
    end

    if ~isempty(models{2})
        assignin('base',motor + "_Gc_2P1Z",models{2});
    end

    if ~isempty(models{3})
        assignin('base',motor + "_Gd_1P0Z",models{3});
    end

    if ~isempty(models{4})
        assignin('base',motor + "_Gd_2P1Z",models{4});
    end

    % Best model
    assignin( ...
        'base', ...
        motor + "_best_tf", ...
        bestPlant);

    % Discrete plant actually used by PID tuning
    assignin( ...
        'base', ...
        motor + "_tuning_plant", ...
        plantForTune);

    % MATLAB duty-percent controller
    assignin( ...
        'base', ...
        motor + "_PIDF_percent", ...
        C_percent);

    % Equivalent STM32 PWM-count controller
    assignin( ...
        'base', ...
        motor + "_PIDF_STM32", ...
        C_STM32);

    %% Closed-loop response
    % Own figure for each motor

    figure( ...
        'Name', ...
        motor + " PIDF Auto-Tune");

    step(closedLoop);

    grid on;

    title( ...
        motor + ...
        " Motor - 100 Hz Trapezoidal PIDF Closed-Loop Response");

    %% Store result

    row = table( ...
        motor, ...
        bestModelName, ...
        bestFit, ...
        string(tunePlantSource), ...
        Kp_percent, ...
        Ki_percent, ...
        Kd_percent, ...
        Tf_percent, ...
        Kp_STM32, ...
        Ki_STM32, ...
        Kd_STM32, ...
        Tf_STM32, ...
        firmwareValid, ...
        closedLoopStable, ...
        'VariableNames',{ ...
        'Motor', ...
        'BestModel', ...
        'BestFit_pct', ...
        'TuningPlant', ...
        'Kp_percent', ...
        'Ki_percent', ...
        'Kd_percent', ...
        'Tf_s', ...
        'Kp_STM32', ...
        'Ki_STM32', ...
        'Kd_STM32', ...
        'Tf_STM32_s', ...
        'FirmwareValid', ...
        'ClosedLoopStable'});

    results = [results; row];

end

%% Final PIDF table

disp(" ");
disp("========================================");
disp("PIDF AUTO-TUNE RESULTS");
disp("========================================");

disp(results);

assignin( ...
    'base', ...
    'PIDF_results', ...
    results);

%% Save for Raspberry Pi updater

outputCSV = fullfile( ...
    pwd, ...
    'pidf_autotune_results.csv');

writetable( ...
    results, ...
    outputCSV);

fprintf("\nSaved PIDF parameters to:\n%s\n",outputCSV);

fprintf( ...
    "\nSTM32 scaling: 1 %% duty = %.4f PWM counts (ARR = %d).\n", ...
    firmwareScale, ...
    pwmARR);

fprintf( ...
    "Controller implementation: Ts = %.3f s, Trapezoidal I, Trapezoidal/Tustin D.\n", ...
    Ts);
