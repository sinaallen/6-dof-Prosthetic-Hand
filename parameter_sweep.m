function parameter_sweep(params, sweep_params, input_disp_range)
    % PARAMETER_SWEEP Performs a parameter sweep over linkage configurations.
    %
    % Inputs:
    %   params          - Structure from mechanism_params() with sweep ranges/steps
    %   sweep_params    - Cell array of parameter names to sweep (e.g., {'R_act', 'B_x'})
    %   input_disp_range - Array of stroke positions (e.g., linspace(0, 15.75, 15))
    %
    % Performs kinematics and force simulations for each valid combination,
    % storing parameter values, PIP angle, proximal angle, and finger tip force.
    % Saves results to 'sweep_results.mat' and plots a 3D scatter.
    % Progress is displayed in the command window.

    % Number of parameters to sweep
    num_sweep_params = length(sweep_params);
    
    % Generate values for each sweep parameter
    values_cell = cell(1, num_sweep_params);
    for i = 1:num_sweep_params
        param = sweep_params{i};
        range = params.sweep.(param).range;
        step = params.sweep.(param).step;
        if range(1) <= range(2)
            values_cell{i} = range(1):step:range(2);
        else
            values_cell{i} = range(1):-step:range(2);
        end
    end
    
    % Calculate total number of combinations
    num_values = cellfun(@numel, values_cell);
    num_combinations = prod(num_values);
    
    % Generate all combinations using ndgrid
    grids = cell(1, num_sweep_params);
    [grids{1:num_sweep_params}] = ndgrid(values_cell{:});
    
    % Initialize results storage
    results = struct('params', {}, 'performance', {});
    
    % Display total number of combinations
    fprintf('Total combinations to run: %d\n', num_combinations);
    
    % Run sweep with interruption handling
    try
        for k = 1:num_combinations
            % Copy params for current combination
            params_current = params;
            
            % Create parameter structure for this combination
            param_values = struct();
            for i = 1:num_sweep_params
                param = sweep_params{i};
                value = grids{i}(k);
                if strcmp(param, 'B_x')
                    params_current.B(1) = value;
                    param_values.B_x = value;
                elseif strcmp(param, 'B_y')
                    params_current.B(2) = value;
                    param_values.B_y = value;
                else
                    params_current.(param) = value;
                    param_values.(param) = value;
                end
            end
            
            % Suppress figures during simulation
            set(0, 'DefaultFigureVisible', 'off');
            
            % Run simulations
            [E_positions, F_positions, phi_history, pip_history] = ...
                kinematics_simulation(params_current, input_disp_range);
            F_Fy = force_analysis(params_current, input_disp_range, ...
                E_positions, F_positions);
            
            % Restore figure visibility
            set(0, 'DefaultFigureVisible', 'on');
            
            % Extract metrics
            phi_max_stroke = phi_history(end);    % Proximal angle at max stroke
            pip_max_stroke = pip_history(end);    % PIP angle at max stroke
            F_Fy_min_stroke = F_Fy(1);            % Finger tip force at min stroke
            
            % Store results with parameters
            results(end+1).params = param_values;
            results(end).performance = [pip_max_stroke, phi_max_stroke, F_Fy_min_stroke];
            fprintf('Progress: %d combinations completed.\n', k);
        end
    catch
        fprintf('Simulation interrupted. Saving partial results...\n');
        save('partial_sweep_results.mat', 'results', 'sweep_params', 'input_disp_range');
        rethrow(lasterror);
    end
    
    % Save full results
    save('sweep_results.mat', 'results', 'sweep_params', 'input_disp_range');
    fprintf('Results saved to sweep_results.mat\n');
    
    % Generate 3D plot if results exist
    if ~isempty(results)
        figure;
        performance_data = vertcat(results.performance);
        scatter3(performance_data(:,1), performance_data(:,2), performance_data(:,3), 'filled');
        xlabel('PIP Angle at Max Stroke (°)');
        ylabel('Proximal Linkage Angle at Max Stroke (°)');
        zlabel('Finger Tip Force at Min Stroke (N)');
        title('Parameter Sweep Results');
        grid on;
    else
        disp('No valid configurations found.');
    end
end