%% CONTROL PROJECT - FINAL VALIDATED IMPLEMENTATION
clear; clc; close all;

%% -------------------------
% 1) Continuous Plant Model
% -------------------------
s = tf('s');
T = 0.1; % Sampling time (sec)

Gp = 4 / ((2*s + 1)*(0.5*s + 1)); % Gp(s)
H  = 1 / (0.05*s + 1);            % H(s)
GHc = Gp * H;

fprintf('Continuous Open-loop DC Gain: %.4f\n', dcgain(GHc));

%% -------------------------
% 2) Discretize Plant with ZOH
% -------------------------
GHd = c2d(GHc, T, 'zoh'); % Pulse transfer of plant+sensor

%% -------------------------
% 3) Final Controller (DESIGNED IN SISO) (scaled it to have ess exactly
% 0.1)
% -------------------------
z = tf('z', T);
Cz_raw = 6.525 * (z - 0.6812) / (z - 0.0739);


disp('Using final discrete controller C(z) from SISO Tool:');
Cz_raw

%% -------------------------
% 4) DC Gain Correction → ESS = 10%
% -------------------------
Ld_raw = Cz_raw * GHd;               % Open-loop before scaling
Cz = Cz_raw;                 % Final Correct Controller


%% -------------------------
% 5) Closed-loop System
% -------------------------
Ld = Cz * GHd;
CLd = feedback(Ld,1);

%% Step Response
figure('Name','Closed-loop Step Response After Compensation');
step(CLd); grid on; title('Closed-loop Step Response (Final Design)');

[y, ~] = step(CLd);
final_val = y(end);
ess = 1 - final_val;

%% Stability Margins
[GM,PM,wgc,wpc] = margin(Ld);

fprintf('\n===== PERFORMANCE RESULTS =====\n');
fprintf('Final Value: %.4f\n', final_val);
fprintf('Steady-State Error: %.4f (Target = 0.10)\n', ess);
fprintf('Gain Margin: %.2f dB\n', 20*log10(GM));
fprintf('Phase Margin: %.2f deg (Target >= 50 deg)\n', PM);
fprintf('Wgc: %.2f rad/s\n', wgc);
fprintf('Wpc: %.2f rad/s\n', wpc);

%% -------------------------
% 6) Bode and Stability Verification
% -------------------------
figure('Name','Open-loop Bode Plot after Compensation');
margin(Ld); grid on;
title('Open-Loop with Final Discrete Controller C(z)');

%% -------------------------
% 7) Show in SISO Tool (Validation Only)
% -------------------------
disp('Opening SISO Tool for verification…');
sisotool(Cz, GHd);
