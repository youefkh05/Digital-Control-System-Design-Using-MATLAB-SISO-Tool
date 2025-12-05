%% CONTROL PROJECT - FIXED: ENSURE CONTROLLER DOES NOT CHANGE DC GAIN
clear; clc; close all;

%% -------------------------
% 1) Continuous Plant Model
% -------------------------
s = tf('s');
T = 0.1; % Sampling time (sec)

Gp = 4 / ((2*s + 1)*(0.5*s + 1)); % Gp(s)
H  = 1 / (0.05*s + 1);            % H(s)
GHc = Gp * H;                     % Continuous open-loop (plant + sensor)

% Compute original steady-state error (Type-0 step input)
Kp_original = dcgain(GHc);
Ess_original = 1/(1 + Kp_original);

fprintf('Original DC Gain (before adding gain): %.6f\n', Kp_original);
fprintf('Original Steady-state Error: %.6f\n', Ess_original);

%% ---------------------------------------------------
% 2) Add Gain to Make Ess = 10% Before Controller
% ---------------------------------------------------
% Target Ess = 0.10 --> Kp_target = (1/Ess) - 1 = 9
Kp_target = 9;
K_required = Kp_target / Kp_original;

fprintf('\nRequired gain multiplier for Ess=10%%: %.6f\n', K_required);

% Apply gain to plant (ONLY HERE!)
GHc = K_required * GHc;

% New steady-state error (should be ≈ 0.10)
Ess_new = 1/(1 + dcgain(GHc));
fprintf('New Steady-state Error after applying plant gain: %.6f (Target = 0.10)\n', Ess_new);

%% -------------------------
% 3) Discretize Plant with ZOH
% -------------------------
GHd = c2d(GHc, T, 'zoh'); % Discrete plant with gain

%% -----------------------------------------------
% 4) Bode Plot & Margins BEFORE Controller
% -----------------------------------------------
figure('Name','Bode BEFORE Controller');
margin(GHd); grid on;
title('Open-Loop BEFORE Controller (Plant + H + Gain)');

[GM0, PM0, wgc0, wpc0] = margin(GHd);

fprintf('\n=== BEFORE CONTROLLER ===\n');
fprintf('Gain Margin: %.2f dB\n', 20*log10(GM0));
fprintf('Phase Margin: %.2f deg\n', PM0);
fprintf('Wgc: %.4f rad/s\n', wgc0);
fprintf('Wpc: %.4f rad/s\n', wpc0);

%% -------------------------
% 5) Final Controller (from SISO design)
%    --- SCALE IT TO HAVE DC GAIN = 1 ---
% -------------------------
z = tf('z', T);

% Controller shape (zero & pole from SISO design)
z0 = 0.6812;
p0 = 0.0739;
Cz_unscaled = (z - z0) / (z - p0);

% Compute scaling so Cz(1) == 1 (unit DC gain)
Kc = 1 / dcgain(Cz_unscaled);
Cz = Kc * Cz_unscaled;

% display controller info
disp('Controller shape (unscaled):');
Cz_unscaled
fprintf('Scaling Kc to enforce Cz(1)=1: %.6f\n', Kc);
disp('Final controller Cz (after scaling to unit DC):');
Cz

fprintf('Cz DC gain check: Cz(1) = %.6f\n', dcgain(Cz));

%% -------------------------
% 6) Closed-loop AFTER Controller
% -------------------------
Ld = Cz * GHd;   % Open-loop with controller
CLd = feedback(Ld, 1);

%% Step Response
figure('Name','Closed-loop Step Response After Compensation');
step(CLd); grid on;
title('Closed-loop Step Response (Final Design)');

[y, ~] = step(CLd);
final_val = y(end);
ess_after = 1 - final_val;

%% Stability Margins After Controller
[GM, PM, wgc, wpc] = margin(Ld);

fprintf('\n===== PERFORMANCE AFTER CONTROLLER =====\n');
fprintf('Final Value: %.6f\n', final_val);
fprintf('Steady-State Error AFTER controller: %.6f (Target = 0.10)\n', ess_after);
fprintf('Open-loop DC gain L(1) = Cz(1)*dcgain(GHd) = %.6f\n', dcgain(Ld));
fprintf('Gain Margin: %.2f dB\n', 20*log10(GM));
fprintf('Phase Margin: %.2f deg (Target >= 50 deg)\n', PM);
fprintf('Wgc: %.4f rad/s\n', wgc);
fprintf('Wpc: %.4f rad/s\n', wpc);

%% -------------------------
% 7) Bode Plot After Controller
% -------------------------
figure('Name','Open-loop With Controller');
margin(Ld); grid on;
title('Open-Loop with Final Discrete Controller C(z)');

%% -------------------------
% 8) SISO Tool Validation (optional)
% -------------------------
disp('Opening SISO Tool for verification…');
sisotool(Cz, GHd);
