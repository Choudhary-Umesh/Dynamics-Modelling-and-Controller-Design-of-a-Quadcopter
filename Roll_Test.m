%% QUADCOPTER ROLL PLANT SIMULATION TRAJECTORY GENERATOR (Figure 2.32)
% 1. CLEAR MEMORY FIRST (Updated safety list to preserve ALL new Table 11.3 variables)
clearvars -except Jr Ixx Iyy Izz Ax Ay Az b d l m g Tm CR w_b;
close all;
clc;

% --- Configuration ---
model_name = 'quad_dynamics'; 
t_sim_end  = 20;               
set_param(model_name, 'StopTime', num2str(t_sim_end));

%% 2. Corrected Roll Imbalance Inputs (Left vs Right side)
% Left motors higher, Right motors lower -> Creates a roll tilt along Y-axis
Omega1_val = 599.99;  % Right Front
Omega2_val = 600.01;  % Left Front  <-- High
Omega3_val = 600.01;  % Left Rear   <-- High
Omega4_val = 599.99;  % Right Rear

%% 3. Run the Simulink Model (Modern Explicit Workspace Mapping)
fprintf('Simulating roll profile...\n');
simIn = Simulink.SimulationInput(model_name);
simIn = simIn.setVariable('Omega1_val', Omega1_val);
simIn = simIn.setVariable('Omega2_val', Omega2_val);
simIn = simIn.setVariable('Omega3_val', Omega3_val);
simIn = simIn.setVariable('Omega4_val', Omega4_val);

simData = sim(simIn);

%% 4. Extract Trajectory Coordinate Data
X = simData.X_out;   
Y = simData.Y_out;   
Z = simData.Z_out;

%% 5. Render Figure 2.32 Graph
figure('Name', 'Figure 2.32 - Roll plant simulation', 'Color', 'w');

% Plot Center of Mass arc track
plot3(X, Y, Z, 'LineWidth', 2.5, 'Color', [0 0.447 0.741]); 

% Layout formatting
grid on;
xlabel('X Position(m)');
ylabel('Y Position(m)');
zlabel('Z Position(m)');
title('Quadcopter Trajectory');
subtitle('Figure 2.32-Roll plant simulation');
view(3);