clc
clear all
close all
fig = 1;

% Make constants accessible for Simulink blocks
global Jr Ixx Iyy Izz Ax Ay Az b d l m g 
global Tm CR w_b % Quan Quan's First-Order Motor/ESC parameters

%% =========================================================================
%% 1. QUADROTOR CONSTANTS (Source: Quan Quan, Table 11.3)
%% =========================================================================
g   = 9.8100;         % Gravitational acceleration (m/s^2)
m   = 1.0230;         % Mass of the Quadrotor (kg)
l   = 0.2223;         % Distance from motor to center of mass (m)

% Moments of Inertia (kg*m^2) measured experimentally via bifilar pendulum
Ixx = 0.0095;         % Quadrotor moment of inertia around X axis 
Iyy = 0.0095;         % Quadrotor moment of inertia around Y axis
Izz = 0.0186;         % Quadrotor moment of inertia around Z axis
Jr  = 6.5e-07;        % Rotor structural inertia (Placeholder retained)

% Actuator / Motor+ESC Parameters (Source: Quan Quan, Table 11.3)
% Replaces Ferry's R_m, L_m with a realistic first-order ESC/Motor model
Tm  = 0.0760;         % Motor/ESC time constant (s)
CR  = 80.5840;        % Throttle-to-speed slope constant (RPM/%)
w_b = 976.2000;       % Base speed constant at zero throttle (RPM)

% Air resistance / Translational Drag coefficients 
% (Set to 0 in Table 11.3 for baseline hovering/low-speed tracking)
Ax  = 0.0;            
Ay  = 0.0;            
Az  = 0.0;             

%% =========================================================================
%% 2. AERODYNAMIC COEFFICIENT CONVERSION
%% =========================================================================
% Quan Quan's dynamometer tests provide the coefficients in terms of RPM^2.
% We multiply by the conversion factor to make them work with rad/s in Simulink.

rpm_to_rads_factor = (30 / pi)^2;

cT_rpm = 1.4865e-07;  % Thrust coefficient (N/RPM^2) [Quan Quan Table 11.3]
cM_rpm = 2.9250e-09;  % Torque coefficient (Nm/RPM^2) [Quan Quan Table 11.3]

b = cT_rpm * rpm_to_rads_factor;  % Converted Thrust factor (N/(rad/s)^2)
d = cM_rpm * rpm_to_rads_factor;  % Converted Drag/Torque factor (Nm/(rad/s)^2)

%% =========================================================================
%% 3. CONTROLLER GAINS & REFERENCE SIGNALS (Quan Quan Tables 11.5 & 11.6)
%% =========================================================================
%% Altitude (Z) Controller Gains
Kp_z = 4.0;   Ki_z = 2.0;   Kd_z = 4.0;

%% Attitude Controller Gains 
% Note: Quan Quan separates this into angle error (Kp) and angular rate error (K_wp, K_wd)
Kp_roll  = 2.0;   K_wp_roll  = 2.0;   K_wd_roll  = 0.8;
Kp_pitch = 2.0;   K_wp_pitch = 2.0;   K_wd_pitch = 0.8;
Kp_yaw   = 3.0;   K_wp_yaw   = 3.0;   K_wd_yaw   = 2.5;

%% Horizontal Position (X, Y) Controller Gains
Kp_Vx = 0.5;   Ki_Vx = 0.05;   Kd_Vx = 0.2;
Kp_Vy = 0.5;   Ki_Vy = 0.05;   Kd_Vy = 0.2;

% Reference Profiles (Baseline set to hover at 3 meters)
t_sim     = 0;
h_ref     = -3;       % Local NED frame (Negative Z is UP)
phi_ref   = 0;
theta_ref = 0;
psi_ref   = 0;
vx_ref    = 0;
vy_ref    = 0;

fprintf('System parameters for Sunnysky A2212 / APC 10x4.5 Quadcopter loaded successfully.\n');