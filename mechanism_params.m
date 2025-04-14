function params = mechanism_params()
    % Proximal linkage parameters
    params.O = [0, 0];                % Proximal pivot (fixed)
    params.proximal_length = 40;      % Proximal linkage length (O->A)
    
    params.R_act = 12;                % Distance from O to actuation joint P
    params.L_drive = 20;              % Driving linkage length (P->E)
    params.alpha_act = 100;           % Fixed angle of O to P to proximal linkage (deg)
    
    % Transfer linkage parameters
    params.B = [0, -8];               % Transfer linkage fixed pivot [x, y]
    params.R_coupling = 7;            % Coupling link length (A->C)
    params.fixed_angle = 170;         % Fixed angle between A->C and distal linkage (deg)
    
    % Distal linkage parameters
    params.distal_length = 20;        % Distal linkage length (A->D)
    
    % Tip linkage parameters
    params.tip_length = 20;           % Tip linkage length (D->F)
    params.tip_fixed_angle = 20;      % Fixed angle downward relative to distal linkage (deg)
    
    % Actuation slider joint parameters
    params.E_y = 14;                  % Fixed y-coordinate for slider joint E
    
    % External force applied at E
    params.F_act_mag = 70;            % Magnitude of force at E (N)
    params.F_act = [-params.F_act_mag, 0];  % Force at E (in -x direction)
    
    % Nominal configuration parameters
    params.phi_nom = 180;             % Nominal proximal angle (deg)
    params.distal_nom = 180;          % Nominal distal linkage angle (deg)
    
    % Sweep parameters
    params.sweep.R_act.range = [12, 15];
    params.sweep.R_act.step = 1;
    params.sweep.L_drive.range = [10, 20];
    params.sweep.L_drive.step = 5;
    params.sweep.alpha_act.range = [100, 110];
    params.sweep.alpha_act.step = 5;
    params.sweep.B_x.range = [0, 6];
    params.sweep.B_x.step = 1;        % Step assumed since not provided
    params.sweep.B_y.range = [-3, -8];
    params.sweep.B_y.step = 1;
    params.sweep.R_coupling.range = [4, 10];
    params.sweep.R_coupling.step = 1;
    params.sweep.fixed_angle.range = [150, 170];
    params.sweep.fixed_angle.step = 10;
    params.sweep.E_y.range = [10, 15];
    params.sweep.E_y.step = 1;
end