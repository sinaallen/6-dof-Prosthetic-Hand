% Load mechanism parameters
params = mechanism_params();

% Define stroke range
input_disp_range = linspace(0, 15.75, 2);

% Run kinematics simulation
[E_positions, F_positions, phi_history, pip_history] = kinematics_simulation(params, input_disp_range);

% Perform force analysis
F_Fy = force_analysis(params, input_disp_range(1:2), E_positions, F_positions);

% Specify parameters to sweep 
sweep_params = {'R_act', 'L_drive', 'alpha_act', 'B_x', 'B_y', ...
                'R_coupling', 'fixed_angle', 'E_y'};

% Run parameter sweep
parameter_sweep(params, sweep_params, input_disp_range);

load('sweep_results.mat', 'results');
params = filter_results(results, 90, 85, 15);