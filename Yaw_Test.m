%% QUADCOPTER YAW HELIX TRAJECTORY GENERATOR (Figure 2.34)
% 1. CLEAR MEMORY FIRST (Updated safety list to preserve ALL new Table 11.3 variables)
clearvars -except Jr Ixx Iyy Izz Ax Ay Az b d l m g Tm CR w_b;
close all;
clc;

% --- Configuration ---
model_name = 'quad_dynamics'; 
t_sim_end  = 20;               

% Set the simulation time dynamically
set_param(model_name, 'StopTime', num2str(t_sim_end));

%% 2. Define High-Spin Yaw Imbalance Inputs (Alternating pairs)
Omega1_val = 600;     % CW Pair
Omega2_val = 605;     % CCW Pair (Increased torque for tight helix curve)
Omega3_val = 600;     % CW Pair
Omega4_val = 605;     % CCW Pair (Increased torque for tight helix curve)

%% 3. Run the Simulink Model (Modern Explicit Workspace Mapping)
fprintf('Simulating yaw spiral profile...\n');
simIn = Simulink.SimulationInput(model_name);
simIn = simIn.setVariable('Omega1_val', Omega1_val);
simIn = simIn.setVariable('Omega2_val', Omega2_val);
simIn = simIn.setVariable('Omega3_val', Omega3_val);
simIn = simIn.setVariable('Omega4_val', Omega4_val);

simData = sim(simIn);

%% 4. Extract Core Coordinate Data
X   = simData.X_out;   Y   = simData.Y_out;   Z   = simData.Z_out;
phi = simData.phi_out; theta = simData.theta_out; psi = simData.psi_out;
num_steps = length(X);

m1_pos = zeros(num_steps, 3);
m2_pos = zeros(num_steps, 3);
m3_pos = zeros(num_steps, 3);
m4_pos = zeros(num_steps, 3);

%% 5. Transform Local Arms to Global 3D Paths (Using hardcoded 45-deg angle)
for i = 1:num_steps
    cph = cos(phi(i));   sph = sin(phi(i));
    cth = cos(theta(i)); sth = sin(theta(i));
    cps = cos(psi(i));   sps = sin(psi(i));
    
    R = [ cps*cth,  cps*sth*sph - sps*cph,  cps*sth*cph + sps*sph;
          sps*cth,  sps*sth*sph + cps*cph,  sps*sth*cph - cps*sph;
             -sth,                cth*sph,                cth*cph ];
             
    motor1_local = [ l*cos(pi/4);  l*sin(pi/4); 0];
    motor2_local = [-l*cos(pi/4);  l*sin(pi/4); 0];
    motor3_local = [-l*cos(pi/4); -l*sin(pi/4); 0];
    motor4_local = [ l*cos(pi/4); -l*sin(pi/4); 0];
    
    m1_pos(i, :) = (R * motor1_local)' + [X(i), Y(i), Z(i)];
    m2_pos(i, :) = (R * motor2_local)' + [X(i), Y(i), Z(i)];
    m3_pos(i, :) = (R * motor3_local)' + [X(i), Y(i), Z(i)];
    m4_pos(i, :) = (R * motor4_local)' + [X(i), Y(i), Z(i)];
end

%% 6. Render Figure 2.34 Graph
figure('Name', 'Figure 2.34 - Yaw plant simulation', 'Color', 'w');
plot3(X, Y, Z, 'LineWidth', 2.5, 'Color', [0 0.447 0.741], 'DisplayName', 'Center of Mass'); 
hold on;
plot3(m1_pos(:,1), m1_pos(:,2), m1_pos(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Motor 1');
plot3(m2_pos(:,1), m2_pos(:,2), m2_pos(:,3), 'k-', 'LineWidth', 2, 'DisplayName', 'Motor 2');
plot3(m3_pos(:,1), m3_pos(:,2), m3_pos(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'Motor 3');
plot3(m4_pos(:,1), m4_pos(:,2), m4_pos(:,3), 'm-', 'LineWidth', 2, 'DisplayName', 'Motor 4');

grid on;
xlabel('X Position(m)');
ylabel('Y Position(m)');
zlabel('Z Position(m)');
title('Quadcopter Trajectory');
subtitle('Figure 2.34-Yaw plant simulation');
legend('Location', 'best');
view(3);