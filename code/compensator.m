% CONTROL PROJECT - AUTOMATED SETUP FOR SISO TOOL
% Uses: sisotool, c2d, d2c
% Reference: Control Project - 2025. See uploaded spec. :contentReference[oaicite:1]{index=1}

clear; close all; clc;

%-------------------------
% 1) Continuous plant
%-------------------------
s = tf('s');

Gp = 4 / ((2*s + 1)*(0.5*s + 1));    % Gp(s)
H  = 1 / (0.05*s + 1);               % H(s)
Lc = Gp * H;                         % open-loop continuous

T = 0.1;                             % sample time (seconds) from project

% Print DC gain of continuous loop
dcgain_continuous = dcgain(Lc);
fprintf('Continuous open-loop DC gain (Lc(0)) = %.6f\n', dcgain_continuous);

%-------------------------
% 2) Required DC loop gain for 10% steady-state error
% For unity feedback: ess = 1/(1 + L(0)) -> want ess = 0.10 -> L(0) = 9
% So required overall L(0) = 9
%-------------------------
required_L0 = 9;
required_K = required_L0 / dcgain_continuous;
fprintf('Scalar gain required (approx) to meet ess=10%%: K_required = %.4f\n', required_K);
% We'll use this K as an initial starting gain for design in SISO Tool

%-------------------------
% 3) Discretize the plant (use ZOH as required in project)
%-------------------------
Lc_with_gain = required_K * Lc;
Gd = c2d(Lc_with_gain, T, 'zoh');    % discrete plant = K * Gp * H discretized
fprintf('Discrete plant created with sample time T = %.3f s\n', T);

% Save plants to workspace for easy access in SISO Tool
Gp_cont = Gp; %#ok<NASGU>
H_cont  = H;  %#ok<NASGU>
L_cont_with_gain = Lc_with_gain; %#ok<NASGU>
L_disc = Gd; %#ok<NASGU>

%-------------------------
% 4) Plot Bode (continuous and discrete) for quick look
%-------------------------
figure('Name','Bode: continuous (with K) vs discrete (zoh)','NumberTitle','off');
subplot(2,1,1);
bode(Lc_with_gain); grid on;
title('Continuous open-loop (with K_{init})');

subplot(2,1,2);
bode(Gd); grid on;
title('Discretized open-loop (zoh)');

%-------------------------
% 5) Launch SISO Design Tool on the discrete plant
%    → You will interactively design a discrete controller Kd(z)
%    Steps to follow inside SISO Tool:
%      - Add a compensator (type: discrete, e.g., PI, lead, lead-lag, or P with zeros/poles)
%      - Adjust gains and pole/zero placements to get Phase Margin >= 50 deg
%      - Ensure low-frequency gain yields final value ~0.9 (or adjust if needed)
%      - Export designed controller to workspace (Controller -> Export -> To Workspace)
%-------------------------
fprintf('\nLaunching SISO Design Tool for the discrete plant...\n');
fprintf('Inside SISO Tool: design a discrete controller Kd(z) to meet specs:\n');
fprintf('  - steady-state error to unit step = 10%% (final value ~0.9)\n');
fprintf('  - Phase margin >= 50 degrees\n\n');
% Launch sisotool with the discrete plant as the plant in the loop
sisotool(Gd);

% After you finish designing in SISO Tool, export controller to workspace with a name e.g. Kd_designed
% The following steps assume you exported the controller to variable "Kd_designed".

% ------------------------
% 6) Validation code (run AFTER you export Kd_designed from SISO Tool)
% ------------------------
% Paste/run the following block after exporting controller:


% Example validation (uncomment and run after exporting Kd_designed from SISO Tool)
if exist('Kd_designed','var') ~= 1
    warning('Controller Kd_designed not found in workspace. Export it from SISO Tool and then run the validation block.');
else
    % Closed-loop discrete (unity feedback)
    Ld = series(Kd_designed, Gd);            % open-loop discrete (controller * plant)
    CLd = feedback(Ld, 1);                   % closed-loop discrete
    
    % Step response
    figure('Name','Closed-loop step (discrete)','NumberTitle','off');
    step(CLd); grid on; title('Closed-loop Step Response (discrete)');

    % Final value (steady-state) and ess
    [y_final, t_final] = step(CLd, 0: T : 50); % simulate for long enough
    final_val = y_final(end);
    ess = 1 - final_val;
    fprintf('Closed-loop final value = %.4f, steady-state error = %.4f (target 0.10)\n', final_val, ess);

    % Phase margin (use margin on the open-loop continuous equivalent or discrete)
    % We'll compute margins on the discrete open-loop Ld (use bode/margin for discrete)
    [Gm,Pm,Wcg,Wcp] = margin(Ld);
    fprintf('Discrete open-loop margins: Pm = %.2f deg (need >= 50 deg)\n', Pm);

    % If you want to convert Kd back to continuous for reporting:
    try
        Kc_from_d = d2c(Kd_designed, 'tustin'); % d2c via Tustin (bilinear)
        fprintf('Converted Kd -> continuous (d2c using tustin). Use this for report if needed.\n');
    catch ME
        warning('d2c conversion failed: %s\n', ME.message);
    end

    % Plot before vs after (continuous reference: before = Lc_without_K, after = with K exported->converted)
    % Get continuous open-loop before (without K) for comparison:
    Lc_before = Gp * H;  % original continuous open-loop (no additional K)
    figure('Name','Step response: before vs after (discrete closed-loop)','NumberTitle','off');
    subplot(2,1,1);
    step(feedback(c2d(Lc_before,T,'zoh'),1)); grid on; title('Before compensation (discrete conversion of original Lc)');

    subplot(2,1,2);
    step(CLd); grid on; title('After compensation (designed Kd)');

    % Save figures (optional)
    saveas(gcf, 'step_before_after.png');
end


% End of script
