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
%   -> if best plant is continuous, discretize plant using bilinear/Tustin at Ts = 0.01 s
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

% Closed-loop targets for each motor.
% Rows correspond to motors = ["WR","WL","BR","BL","CV"].
%
% Edit these values independently as required.
settlingTarget_s_all = [
    0.10;   % WR
    0.10;   % WL
    0.10;   % BR
    0.10;   % BL
    0.10    % CV
];

overshootTarget_pct_all = [
    2.0;    % WR
    2.0;    % WL
    2.0;    % BR
    2.0;    % BL
    2.0     % CV
];

% Model-selection override for PIDF tuning.
% Rows correspond to motors = ["WR","WL","BR","BL","CV"].
%
% Allowed values:
%   "AUTO"    -> tune the model with the highest validation fit
%   "C_1P0Z"  -> force continuous 1 pole / 0 zero
%   "C_2P1Z"  -> force continuous 2 pole / 1 zero
%   "D_1P0Z"  -> force discrete   1 pole / 0 zero
%   "D_2P1Z"  -> force discrete   2 pole / 1 zero
modelOverride_all = [
    "AUTO";   % WR
    "AUTO";   % WL
    "AUTO";   % BR
    "AUTO";   % BL
    "AUTO"    % CV
];

% Search a range of robust PIDF designs.
% The 100 Hz sample rate gives a sampling angular frequency of 2*pi/Ts.
% Keep the requested crossover well below it.
sampleAngularFrequency = 2*pi/Ts;
wcCandidates = linspace(5,0.15*sampleAngularFrequency,60);
phaseMarginCandidates = [65 70 75 80 85];

results = table();

%% Process all motors

for m = 1:numel(motors)

    motor = motors(m);

    % Motor-specific closed-loop requirements
    settlingTarget_s = settlingTarget_s_all(m);
    overshootTarget_pct = overshootTarget_pct_all(m);
    modelOverride = upper(strtrim(modelOverride_all(m)));

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
    fprintf("Settling-time target: < %.3f s\n",settlingTarget_s);
    fprintf("Overshoot target    : < %.2f %%\n",overshootTarget_pct);
    fprintf("Model override      : %s\n",modelOverride);

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

    %% Choose model for PIDF tuning

    allowedModelOverrides = [
        "AUTO"
        "C_1P0Z"
        "C_2P1Z"
        "D_1P0Z"
        "D_2P1Z"
    ];

    if ~any(modelOverride == allowedModelOverrides)

        error( ...
            "%s: invalid model override '%s'. Allowed: AUTO, C_1P0Z, C_2P1Z, D_1P0Z, D_2P1Z.", ...
            motor, ...
            modelOverride);

    end

    % Highest-fit model is still recorded even when an override is used.
    [autoBestFit,autoBestIndex] = max(fits);

    if ~isfinite(autoBestFit)

        warning("No valid model found for %s.",motor);
        continue;

    end

    autoBestModelName = modelNames(autoBestIndex);

    if modelOverride == "AUTO"

        selectedModelIndex = autoBestIndex;
        modelSelectionMode = "AUTO";

    else

        selectedModelIndex = find(modelNames == modelOverride,1);
        modelSelectionMode = "OVERRIDE";

        if isempty(selectedModelIndex)

            error( ...
                "%s: override model '%s' was not found.", ...
                motor, ...
                modelOverride);

        end

        if ~isfinite(fits(selectedModelIndex)) || isempty(models{selectedModelIndex})

            warning( ...
                "%s: override model %s is unavailable/unstable and cannot be tuned.", ...
                motor, ...
                modelOverride);

            continue;

        end

    end

    bestIndex = selectedModelIndex;
    bestFit = fits(bestIndex);

    bestModelName = modelNames(bestIndex);
    bestModelLongName = modelLongNames(bestIndex);

    bestIDModel = models{bestIndex};
    bestPlant = tf(bestIDModel);

    fprintf( ...
        "\nAUTO BEST MODEL: %s = %.2f %%\n", ...
        modelLongNames(autoBestIndex), ...
        autoBestFit);

    if modelSelectionMode == "OVERRIDE"

        fprintf( ...
            "TUNING OVERRIDE: %s = %.2f %%\n", ...
            bestModelLongName, ...
            bestFit);

    else

        fprintf( ...
            "TUNING MODEL: %s = %.2f %%\n", ...
            bestModelLongName, ...
            bestFit);

    end

    disp(bestPlant);

    %% ============================================================
    %  PREPARE PLANT FOR THE ACTUAL 100 Hz DIGITAL CONTROLLER
    % =============================================================

    if bestPlant.Ts == 0

        % Discretize the identified continuous plant with the bilinear
        % (Tustin) transform at the STM32 controller sample time.
        plantForTune = c2d( ...
            bestPlant, ...
            Ts, ...
            'tustin');

        tunePlantSource = "Continuous model -> Tustin at 100 Hz";

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

    %% ============================================================
    %  TARGET-BASED PIDF TUNING
    % =============================================================
    %
    % pidtune does not directly accept settling-time and overshoot
    % constraints. Therefore search crossover frequency and phase margin,
    % validate each resulting digital closed loop in the time domain, and
    % select the least aggressive design that satisfies both targets.

    selectedController = [];
    selectedTuneInfo = [];
    selectedClosedLoop = [];

    selectedSettlingTime = Inf;
    selectedOvershoot = Inf;
    selectedRiseTime = Inf;
    selectedCrossover = Inf;
    selectedTargetPM = NaN;
    selectedActualPM = NaN;

    foundTargetDesign = false;

    bestFallbackScore = Inf;
    fallbackController = [];
    fallbackTuneInfo = [];
    fallbackClosedLoop = [];
    fallbackSettlingTime = Inf;
    fallbackOvershoot = Inf;
    fallbackRiseTime = Inf;
    fallbackCrossover = NaN;
    fallbackTargetPM = NaN;
    fallbackActualPM = NaN;

    for pmTarget = phaseMarginCandidates

        tuneOptions = pidtuneOptions( ...
            'PhaseMargin',pmTarget, ...
            'DesignFocus','reference-tracking');

        for wcTarget = wcCandidates

            try

                [Ctry,infoTry] = pidtune( ...
                    plantForTune, ...
                    Ctemplate, ...
                    wcTarget, ...
                    tuneOptions);

                CLtry = feedback( ...
                    Ctry * plantForTune, ...
                    1);

                if ~isstable(CLtry)
                    continue;
                end

                % Evaluate the actual sampled closed-loop step response.
                S = stepinfo( ...
                    CLtry, ...
                    'SettlingTimeThreshold',0.02);

                Ts_try = S.SettlingTime;
                OS_try = S.Overshoot;
                Tr_try = S.RiseTime;

                if ~isfinite(Ts_try) || ~isfinite(OS_try)
                    continue;
                end

                meetsTargets = ...
                    Ts_try < settlingTarget_s && ...
                    OS_try < overshootTarget_pct;

                % Normalized target violation.
                settlingViolation = ...
                    max(0,Ts_try/settlingTarget_s - 1);

                overshootViolation = ...
                    max(0,OS_try/overshootTarget_pct - 1);

                fallbackScore = ...
                    settlingViolation^2 + ...
                    overshootViolation^2 + ...
                    1e-4*wcTarget;

                if fallbackScore < bestFallbackScore

                    bestFallbackScore = fallbackScore;

                    fallbackController = Ctry;
                    fallbackTuneInfo = infoTry;
                    fallbackClosedLoop = CLtry;

                    fallbackSettlingTime = Ts_try;
                    fallbackOvershoot = OS_try;
                    fallbackRiseTime = Tr_try;

                    fallbackCrossover = wcTarget;
                    fallbackTargetPM = pmTarget;

                    if isfield(infoTry,'PhaseMargin')
                        fallbackActualPM = infoTry.PhaseMargin;
                    else
                        fallbackActualPM = NaN;
                    end

                end

                if meetsTargets

                    % Prefer the lowest crossover frequency that meets both
                    % requirements. This reduces noise sensitivity/control
                    % effort compared with simply taking the fastest design.
                    if ~foundTargetDesign || ...
                       wcTarget < selectedCrossover || ...
                       (abs(wcTarget-selectedCrossover) < 1e-12 && ...
                        OS_try < selectedOvershoot)

                        foundTargetDesign = true;

                        selectedController = Ctry;
                        selectedTuneInfo = infoTry;
                        selectedClosedLoop = CLtry;

                        selectedSettlingTime = Ts_try;
                        selectedOvershoot = OS_try;
                        selectedRiseTime = Tr_try;

                        selectedCrossover = wcTarget;
                        selectedTargetPM = pmTarget;

                        if isfield(infoTry,'PhaseMargin')
                            selectedActualPM = infoTry.PhaseMargin;
                        else
                            selectedActualPM = NaN;
                        end

                    end

                end

            catch ME

                % Some crossover/phase-margin combinations may be infeasible
                % for a particular identified plant.
                fprintf( ...
                    "Tune skipped: wc=%.2f rad/s, PM=%g deg: %s\n", ...
                    wcTarget, ...
                    pmTarget, ...
                    ME.message);

            end

        end

    end

    %% Select target-meeting design or closest feasible fallback

    if foundTargetDesign

        C_percent = selectedController;
        tuneInfo = selectedTuneInfo;
        closedLoop = selectedClosedLoop;

        settlingTime_s = selectedSettlingTime;
        overshoot_pct = selectedOvershoot;
        riseTime_s = selectedRiseTime;

        selectedWc = selectedCrossover;
        selectedPMTarget = selectedTargetPM;
        selectedPMActual = selectedActualPM;

        meetsTargets = true;

    else

        if isempty(fallbackController)

            warning( ...
                "%s: no stable PIDF design was found in the search range.", ...
                motor);

            continue;

        end

        C_percent = fallbackController;
        tuneInfo = fallbackTuneInfo;
        closedLoop = fallbackClosedLoop;

        settlingTime_s = fallbackSettlingTime;
        overshoot_pct = fallbackOvershoot;
        riseTime_s = fallbackRiseTime;

        selectedWc = fallbackCrossover;
        selectedPMTarget = fallbackTargetPM;
        selectedPMActual = fallbackActualPM;

        meetsTargets = false;

        warning( ...
            "%s could not satisfy Ts < %.3f s and OS < %.2f %% within the tuning search. Using closest stable design.", ...
            motor, ...
            settlingTarget_s, ...
            overshootTarget_pct);

    end

    %% Confirm controller implementation and achieved performance

    fprintf("\nPIDF TUNING IMPLEMENTATION\n");
    fprintf("----------------------------------------\n");
    fprintf("Controller sample time  = %.6f s\n",C_percent.Ts);
    fprintf("Integral formula         = %s\n",C_percent.IFormula);
    fprintf("Derivative formula       = %s\n",C_percent.DFormula);
    fprintf("Tuning plant             = %s\n",tunePlantSource);
    fprintf("Selected crossover       = %.4f rad/s\n",selectedWc);
    fprintf("Target phase margin      = %.2f deg\n",selectedPMTarget);
    fprintf("Actual phase margin      = %.2f deg\n",selectedPMActual);

    fprintf("\nCLOSED-LOOP TARGETS\n");
    fprintf("----------------------------------------\n");
    fprintf("Settling time target     < %.3f s\n",settlingTarget_s);
    fprintf("Settling time achieved   = %.5f s\n",settlingTime_s);
    fprintf("Overshoot target         < %.2f %%\n",overshootTarget_pct);
    fprintf("Overshoot achieved       = %.3f %%\n",overshoot_pct);
    fprintf("Rise time                = %.5f s\n",riseTime_s);
    fprintf("Targets satisfied        = %s\n",string(meetsTargets));

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

    %% Closed-loop stability using selected digital controller

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
        string(modelSelectionMode), ...
        string(modelOverride), ...
        autoBestModelName, ...
        autoBestFit, ...
        bestModelName, ...
        bestFit, ...
        string(tunePlantSource), ...
        selectedWc, ...
        selectedPMTarget, ...
        selectedPMActual, ...
        settlingTarget_s, ...
        overshootTarget_pct, ...
        settlingTime_s, ...
        overshoot_pct, ...
        riseTime_s, ...
        meetsTargets, ...
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
        'ModelSelectionMode', ...
        'ModelOverride', ...
        'AutoBestModel', ...
        'AutoBestFit_pct', ...
        'TunedModel', ...
        'TunedModelFit_pct', ...
        'TuningPlant', ...
        'Crossover_rad_s', ...
        'TargetPhaseMargin_deg', ...
        'ActualPhaseMargin_deg', ...
        'SettlingTarget_s', ...
        'OvershootTarget_pct', ...
        'SettlingTime_s', ...
        'Overshoot_pct', ...
        'RiseTime_s', ...
        'MeetsTargets', ...
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

rawDataDir = fullfile(pwd,'raw_data');

if ~exist(rawDataDir,'dir')
    mkdir(rawDataDir);
end

outputCSV = fullfile( ...
    rawDataDir, ...
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

fprintf("\nMotor-specific closed-loop targets and model overrides:\n");
for m = 1:numel(motors)
    fprintf( ...
        "  %s: settling time < %.3f s, overshoot < %.2f %%, model = %s\n", ...
        motors(m), ...
        settlingTarget_s_all(m), ...
        overshootTarget_pct_all(m), ...
        modelOverride_all(m));
end
