clc
clear all
close all
fig = 1;

% Make constants accessible if still utilizing global blocks
global Jr Ixx Iyy Izz Ax Ay Az b d l m g 
global R_m L_m Kt Kemf b_m % Ferry's specific DC motor parameters

%% =========================================================================
%% 1. QUADROTOR CONSTANTS (Updated from Ferry's Thesis)
%% =========================================================================
g   = 9.8100;         % Gravitational acceleration (m/s^2)
m   = 0.284;          % Mass of the Quadrotor (kg) [Ferry Section 2.1.3]
l   = 0.225;          % Distance from motor to center (m) [Note: Estimated for 0.284kg frame, adjust as needed]

% Moments of Inertia (kg*m^2)
% [Note: Ferry calculates these dynamically based on Appendix A.1. 
% These are typical placeholders for a 0.284kg drone]
Ixx = 0.005;          % Quadrotor moment of inertia around X axis 
Iyy = 0.005;          % Quadrotor moment of inertia around Y axis
Izz = 0.010;          % Quadrotor moment of inertia around Z axis
Jr  = 6.5e-07;        % Propeller/Rotor structural inertia (kg*m^2) [Ferry Table 2.9]

% Actuator / DC Motor Parameters from Ferry Table 2.9
R_m  = 0.117;         % Electrical resistance (Ohms)
L_m  = 1.17e-4;       % Electrical inductance (H)
Kt   = 0.00255;       % Motor torque coefficient (Nm/A)
Kemf = 0.00255;       % Back electromotive force coef. (Vs/rad)
b_m  = 2.415e-6;      % Motor damping coefficient (Nms)

% Air resistance / Translational Drag coefficients [Ferry Table 2.3]
Ax  = 0.1;            % Air resistance in X axis (kg/s)
Ay  = 0.1;            % Air resistance in Y axis (kg/s)
Az  = 0.1;            % Air resistance in Z axis (kg/s)

%% =========================================================================
%% 2. AERODYNAMIC COEFFICIENT CONVERSION
%% =========================================================================
% Ferry's dynamometer tests provide the coefficients directly in terms of 
% angular velocity (rad/s)^2, so no RPM-to-rad/s conversion factor is needed.

b = 1.144e-08;        % Thrust coefficient (N/(rad/s)^2) [Ferry Figure 2.8]
d = 9.940e-10;        % Drag/Torque coefficient (Nm/(rad/s)^2) [Ferry Figure 2.9]

%% =========================================================================
%% 3. CONTROLLER GAINS & REFERENCE SIGNALS (Ferry Table 4.3 & Iteration 4)
%% =========================================================================
%% Altitude Controller (PI-D structure)
Kp_z = 0;   Ki_z = 0;   Kd_z = 0;

%% Attitude Controller (PI-D structure)
Kp_roll  = 2.0;   Ki_roll  = 2.0;   Kd_roll  = 0.1;
Kp_pitch = 2.0;   Ki_pitch = 2.0;   Kd_pitch = 0.1;
Kp_yaw   = 0.1;   Ki_yaw   = 0.1;   Kd_yaw   = 0.0;

%% Position Controller 
% (Ferry does not specify XY position PID gains in his manual tuning section)
Kp_Vx = 0;   Ki_Vx = 0;   Kd_Vx = 0;
Kp_Vy = 0;   Ki_Vy = 0;   Kd_Vy = 0;

% Reference Profiles
t_sim     = 0;
h_ref     = 0;
phi_ref   = 0;
theta_ref = 0;
psi_ref   = 0;
vx_ref    = 0;
vy_ref    = 0;

fprintf('System parameters from Quad_Constant_Ferry loaded successfully.\n');