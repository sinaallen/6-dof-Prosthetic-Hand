function [E_positions, F_positions, phi_history, pip_history] = kinematics_simulation(params, input_disp_range)
    % Extract parameters for clarity
    O = params.O;
    proximal_length = params.proximal_length;
    R_act = params.R_act;
    L_drive = params.L_drive;
    alpha_act = params.alpha_act;
    B = params.B;
    R_coupling = params.R_coupling;
    fixed_angle = params.fixed_angle;
    distal_length = params.distal_length;
    tip_length = params.tip_length;
    tip_fixed_angle = params.tip_fixed_angle;
    E_y = params.E_y;
    phi_nom = params.phi_nom;
    distal_nom = params.distal_nom;

    % Compute nominal positions
    P_nom = O + R_act * [cosd(phi_nom - alpha_act), sind(phi_nom - alpha_act)];
    A_nom = O + proximal_length * [cosd(phi_nom), sind(phi_nom)];
    term_nom = L_drive^2 - (E_y - P_nom(2))^2;
    if term_nom < 0
        error('Invalid nominal configuration: L_drive is too short.');
    end
    E_home_x = P_nom(1) + sqrt(term_nom);  % Home slider x-position

    % Compute nominal coupling joint C_nom
    angleAC_nom = distal_nom - fixed_angle;
    C_nom = A_nom + R_coupling * [cosd(angleAC_nom), sind(angleAC_nom)];
    L_transfer = norm(C_nom - B);  % Transfer linkage length

    % Prepare storage
    numSteps = length(input_disp_range);
    E_positions = zeros(2, numSteps);
    F_positions = zeros(2, numSteps);
    phi_history = zeros(1, numSteps);
    pip_history = zeros(1, numSteps);
    phi_prev = phi_nom;  % Initial guess for phi

    % Prepare figure for animation ****COMMENT OUT BEFORE DOING A SWEEP
    % figure('Color', [1 1 1]);
    % axis equal; grid on; hold on;
    % xlabel('X'); ylabel('Y');
    % title('Mechanism Kinematics Animation');
    % axis([-80 30 -75 20]);

    for k = 1:numSteps
        input_disp = input_disp_range(k);
        E_desired_x = E_home_x - input_disp;

        % Solve for phi using fzero
        f = @(phi) R_act * cosd(phi - alpha_act) + sqrt(L_drive^2 - (E_y - R_act * sind(phi - alpha_act))^2) - E_desired_x;
        phi = fzero(f, phi_prev, optimset('Display', 'off'));
        phi_prev = phi;

        % Compute positions
        P = O + R_act * [cosd(phi - alpha_act), sind(phi - alpha_act)];
        A = O + proximal_length * [cosd(phi), sind(phi)];

        % Compute C (transfer linkage)
        vecAB = B - A;
        d_AB = norm(vecAB);
        a_trans = (R_coupling^2 - L_transfer^2 + d_AB^2) / (2 * d_AB);
        h_trans = sqrt(max(R_coupling^2 - a_trans^2, 0));
        basePoint_trans = A + a_trans * vecAB / d_AB;
        perp_trans = [-vecAB(2), vecAB(1)] / d_AB;
        C1 = basePoint_trans + h_trans * perp_trans;
        C2 = basePoint_trans - h_trans * perp_trans;
        C = (C1(2) >= C2(2)) * C1 + (C1(2) < C2(2)) * C2;  % Choose higher y

        % Compute D (distal linkage)
        angleAC = atan2d(C(2) - A(2), C(1) - A(1));
        distal_angle = angleAC + fixed_angle;
        D = A + distal_length * [cosd(distal_angle), sind(distal_angle)];

        % Compute F (tip linkage)
        F = D + tip_length * [cosd(distal_angle + tip_fixed_angle), sind(distal_angle + tip_fixed_angle)];

        % Compute PIP angle
        raw_diff = mod(distal_angle - phi, 360);
        pip_angle = min(raw_diff, 360 - raw_diff);

        % Compute E
        E = [E_desired_x, E_y];

        % Store positions and angles
        E_positions(:, k) = E';
        F_positions(:, k) = F';
        phi_history(k) = phi - 180;
        pip_history(k) = pip_angle;

        % Plot current configuration ****COMMENT OUT BEFORE DOING A SWEEP
        % cla;
        % hold on;
        % plot(O(1), O(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        % plot(B(1), B(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        % plot([P(1) E(1)], [P(2) E(2)], 'c-', 'LineWidth', 3);
        % plot(E(1), E(2), 'co', 'MarkerSize', 6, 'MarkerFaceColor', 'c');
        % plot(P(1), P(2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
        % plot([O(1) A(1)], [O(2) A(2)], 'b-', 'LineWidth', 3);
        % plot(A(1), A(2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        % t = linspace(0, 2*pi, 50);
        % plot(A(1) + R_coupling * cos(t), A(2) + R_coupling * sin(t), 'm:');
        % plot([B(1) C(1)], [B(2) C(2)], 'r-', 'LineWidth', 3);
        % plot(C(1), C(2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        % plot([A(1) D(1)], [A(2) D(2)], 'g-', 'LineWidth', 3);
        % plot(D(1), D(2), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        % plot([D(1) F(1)], [D(2) F(2)], 'k-', 'LineWidth', 3);
        % plot(F(1), F(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
        % text(O(1)-5, O(2)-5, 'O');
        % text(P(1)-5, P(2)+2, 'P');
        % text(E(1)+2, E(2)+2, 'E');
        % text(A(1)-5, A(2)-5, 'A');
        % text(B(1)+2, B(2)-2, 'B');
        % text(C(1)+2, C(2)+2, 'C');
        % text(D(1)-5, D(2), 'D');
        % text(F(1)-5, F(2), 'F');
        % title(sprintf('Input disp = %.2f, PIP angle = %.2f°', input_disp, pip_angle));
        % drawnow;
        % pause(0.05);
    end

    % Plot PIP angle vs. proximal linkage angle ****COMMENT OUT BEFORE DOING A SWEEP
    % figure('Color', [1 1 1]);
    % plot(phi_history, pip_history, 'b-o', 'LineWidth', 2);
    % xlabel('Proximal Linkage Angle (°)');
    % ylabel('PIP Angle (°)');
    % title('PIP Angle vs. Proximal Linkage Angle');
    % grid on;
end