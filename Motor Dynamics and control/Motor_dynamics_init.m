clc
clear all
close all

%% =========================================================================
%% 4. MOTOR DYNAMICS PARAMETERS (Based on Ferry Table 2.9)
%% =========================================================================
% These parameters model the internal electrical circuit of the brushless DC motor

J_m   = 6.5e-7;      % Motor rotor moment of inertia (kg*m^2) [1]
b_m   = 2.415e-6;    % Motor damping coefficient (Nms) - renamed from 'b' to avoid conflict [1]
K_emf = 0.00255;     % Back electromotive force coefficient (Vs/rad) [1]
K_t   = 0.00255;     % Torque coefficient (Nm/A) [1]
R     = 0.117;       % Electrical resistance (Ohms) [1]
L     = 1.17e-4;     % Electrical Inductance (H) [1]

fprintf('Ferry DC motor dynamics parameters loaded successfully.\n');