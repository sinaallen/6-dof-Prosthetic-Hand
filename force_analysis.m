function F_Fy = force_analysis(params, input_disp_range, E_positions, F_positions)
    % Extract parameters
    F_act_mag = params.F_act_mag;
    O = params.O;

    % Initialize reaction force array
    numSteps = length(input_disp_range);
    F_Fy = zeros(1, numSteps);

    for k = 1:numSteps
        x_F = F_positions(1, k);
        y_E = E_positions(2, k);  % y_E is constant

        if abs(x_F) < 1e-6
            warning('x_F near zero at step %d; F_Fy set to NaN.', k);
            F_Fy(k) = NaN;
        else
            % Compute F_Fy using moment balance: F_Fy = -F_act_mag * y_E / x_F
            F_Fy(k) = -F_act_mag * y_E / x_F;
        end
    end

    % Plot reaction force vs. stroke position ****COMMENT OUT BEFORE DOING A SWEEP
    % figure('Color', [1 1 1]);
    % plot(input_disp_range, F_Fy, 'r-', 'LineWidth', 2);
    % xlabel('Stroke Position (input_disp)');
    % ylabel('Vertical Reaction Force at F (N)');
    % title('Vertical Reaction Force at F vs. Stroke Position');
    % grid on;
end