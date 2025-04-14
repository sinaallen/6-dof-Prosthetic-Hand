function closest_params = filter_results(results, target_pip, target_proximal, target_force)
    % FILTER_RESULTS Finds the mechanism closest to the target performance metrics.
    %
    % Inputs:
    %   results         - Structure array containing simulation results
    %   target_pip      - Target maximum PIP angle at max stroke (degrees)
    %   target_proximal - Target maximum proximal linkage angle at max stroke (degrees)
    %   target_force    - Target maximum finger tip force at min stroke (Newtons)
    %
    % Outputs:
    %   closest_params  - Structure containing parameters of the closest mechanism

    % Initialize variables
    min_error = Inf;  % Start with infinite error to ensure any real error is smaller
    closest_params = struct();  % Empty structure to store the best match

    % Loop through all mechanisms in the results array
    for i = 1:length(results)
        % Extract performance metrics for the current mechanism
        pip_angle = results(i).performance(1);      % PIP angle at max stroke
        proximal_angle = results(i).performance(2); % Proximal angle at max stroke
        force = results(i).performance(3);          % Finger tip force at min stroke

        % Calculate the Euclidean distance (error) from target values
        error = sqrt((pip_angle - target_pip)^2 + ...
                     (proximal_angle - target_proximal)^2 + ...
                     (force - target_force)^2);

        % Update the closest mechanism if this error is smaller
        if error < min_error
            min_error = error;
            closest_params = results(i).params;  % Store the parameters of this mechanism
        end
    end

    % Check if a valid mechanism was found
    if isinf(min_error)
        disp('No valid mechanisms found in the results.');
    else
        disp('Closest mechanism parameters found:');
        disp(closest_params);
    end
end