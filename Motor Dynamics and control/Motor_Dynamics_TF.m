%% DC MOTOR DATA PREPARATION FOR SYSTEM IDENTIFICATION
clearvars;
close all;
clc;

% --- 1. Define Model Name ---
model_name = 'Motor_Dynamics'; % Make sure this matches your exact Simulink file name!

% --- 2. Input DC Motor Parameters ---
R     = 0.6;     
L     = 0.002;   
K_t   = 0.025;   
K_emf = 0.025;  
J_m   = 5e-5;    
b_m   = 1.5e-4;  

% --- 3. Configure and Run Simulation ---
t_sim_end = 10; 
set_param(model_name, 'StopTime', num2str(t_sim_end));

fprintf('Running Simulink model to collect data...\n');
simIn = Simulink.SimulationInput(model_name);
simIn = simIn.setVariable('R', R);
simIn = simIn.setVariable('L', L);        
simIn = simIn.setVariable('K_t', K_t);
simIn = simIn.setVariable('K_emf', K_emf);
simIn = simIn.setVariable('J_m', J_m);    
simIn = simIn.setVariable('b_m', b_m);

% FIXED: Removed the duplicated second sim(simIn) line here
simData = sim(simIn); 

%% --- 4. Extract Data for System Identification Toolbox ---
fprintf('Formatting workspace arrays...\n');
velocity_log = get(simData, 'Velocity');
voltage_log  = get(simData, 'Voltage');

% Convert timeseries structures into flat 1D workspace arrays
Velocity = velocity_log.Data;   
Voltage  = voltage_log.Data;    

% Calculate sample time dynamically
time_array = velocity_log.Time;
sample_time = time_array(2) - time_array(1); 

fprintf('\n=== SUCCESS ===\n');
fprintf('Variables ''Voltage'' and ''Velocity'' are now in your workspace.\n');
fprintf('Use Sample Time: %.4f inside the systemIdentification app.\n', sample_time);