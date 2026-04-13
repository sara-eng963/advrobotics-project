function DrawDeltaRobot(theta, params)
% DRAWDELTAROBOT Visualization with Base at TOP, Platform at BOTTOM.
%   Motors mounted at MIDPOINTS of base triangle edges.
%   Angles measured from HORIZONTAL:
%     - Positive angles: arms point INWARD and DOWNWARD
%     - Negative angles: arms point OUTWARD and UPWARD

    if nargin < 2
        error('DrawDeltaRobot requires theta and params inputs');
    end
    
    theta = theta(:);
    
    % Validate input angles
    for i = 1:3
        if theta(i) < params.thetaLimits(i,1) || theta(i) > params.thetaLimits(i,2)
            error('theta(%d) = %.2f rad (%.1f°) is outside valid range [%.1f°, %.1f°]', ...
                  i, theta(i), rad2deg(theta(i)), ...
                  rad2deg(params.thetaLimits(i,1)), rad2deg(params.thetaLimits(i,2)));
        end
    end
    
    Rb = params.Rb; Rp = params.Rp; L = params.L; l = params.l;
    
    %% 1. Compute Geometry
    % Triangle vertices (corners of base)
    vertices = zeros(3,3);
    for i = 1:3
        angle = (i-1) * 120 * pi/180;
        vertices(i,:) = [Rb * cos(angle), Rb * sin(angle), 0];
    end
    
    % Motor positions at EDGE MIDPOINTS
    A = zeros(3,3);
    for i = 1:3
        next = mod(i, 3) + 1;
        A(i,:) = (vertices(i,:) + vertices(next,:)) / 2;
    end
    
    % Direction from each motor: 
    % - For positive angles: TOWARD center (inward)
    % - For negative angles: AWAY from center (outward)
    B = zeros(3,3);
    for i = 1:3
        % Unit vector from motor to center
        to_center = -[A(i,1), A(i,2)] / norm(A(i,1:2));
        
        % For negative angles, reverse direction (point outward)
        if theta(i) >= 0
            direction = to_center;  % Point INWARD
        else
            direction = -to_center; % Point OUTWARD
        end
        
        % Distance along the direction (horizontal projection)
        dist = L * cos(abs(theta(i)));
        
        % Elbow position
        B(i,1) = A(i,1) + direction(1) * dist;
        B(i,2) = A(i,2) + direction(2) * dist;
        
        % Z coordinate: positive = downward, negative = upward
        % Positive angles: sin(+) = positive (downward)
        % Negative angles: sin(-) = negative (upward)
        B(i,3) = L * sin(theta(i));
    end
    
    % Platform Center (using forward kinematics)
    P = deltaFK(theta(1), theta(2), theta(3), params);
    x = P(1); y = P(2); z = abs(P(3));
    
    % Platform Joints (at BOTTOM, vertices of platform triangle)
    C = zeros(3,3);
    for i = 1:3
        angle = (i-1) * 120 * pi/180;
        C(i,:) = [x + Rp * cos(angle), y + Rp * sin(angle), z];
    end

    %% 2. Create Figure
    fig = figure('Name', 'Delta Robot', 'NumberTitle', 'off', ...
                 'WindowStyle', 'normal', 'Color', 'w');
    set(fig, 'Position', [100, 100, 1000, 800]);
    movegui(fig, 'center');
    
    ax = axes('Parent', fig);
    hold(ax, 'on'); 
    grid(ax, 'on'); 
    axis(ax, 'equal'); 
    box(ax, 'on');
    
    set(ax, 'ZDir', 'reverse');
    view(ax, 3);
    
    xlabel(ax, 'X [m]'); 
    ylabel(ax, 'Y [m]'); 
    zlabel(ax, 'Z [m]');
    
    % Determine arm configuration for title
    if all(theta >= 0)
        config = '▼ Arms: INWARD & DOWNWARD ▼';
    elseif all(theta <= 0)
        config = '▲ Arms: OUTWARD & UPWARD ▲';
    else
        config = 'Arms: Mixed Configuration';
    end
    
    title(ax, sprintf('Delta Robot\nAngles: [%.1f°, %.1f°, %.1f°]\n%s', ...
          rad2deg(theta(1)), rad2deg(theta(2)), rad2deg(theta(3)), config));

    colors = {'r', 'g', 'b'};
    
    %% 3. Draw Elements
    
    % --- BASE TRIANGLE (dashed outline) ---
    plot3(ax, vertices([1:3 1],1), vertices([1:3 1],2), vertices([1:3 1],3), ...
          'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % --- MOTORS (at edge midpoints) ---
    plot3(ax, A(:,1), A(:,2), A(:,3), 'ko', 'MarkerSize', 14, 'MarkerFaceColor', 'k', ...
          'DisplayName', 'Motors');
    
    % Label motors
    for i = 1:3
        text(ax, A(i,1), A(i,2), A(i,3)+0.005, sprintf('M%d', i), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
    end
    
    % --- UPPER ARMS ---
    for i = 1:3
        plot3(ax, [A(i,1), B(i,1)], [A(i,2), B(i,2)], [A(i,3), B(i,3)], ...
            colors{i}, 'LineWidth', 5, 'DisplayName', sprintf('Arm %d', i));
    end
    
    % --- ELBOW JOINTS (spheres) ---
    for i = 1:3
        [X, Y, Z] = sphere(20);
        radius = 0.008;
        X = X * radius + B(i,1);
        Y = Y * radius + B(i,2);
        Z = Z * radius + B(i,3);
        
        switch colors{i}
            case 'r'
                rgb = [0.9 0.2 0.2];
            case 'g'
                rgb = [0.2 0.9 0.2];
            case 'b'
                rgb = [0.2 0.2 0.9];
        end
        
        surf(ax, X, Y, Z, 'FaceColor', rgb, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
        text(ax, B(i,1), B(i,2), B(i,3)-0.008*sign(B(i,3)), sprintf('E%d', i), ...
             'FontSize', 7, 'Color', colors{i});
    end
    
    % --- LOWER RODS (elbow to platform) ---
    for i = 1:3
        plot3(ax, [B(i,1), C(i,1)], [B(i,2), C(i,2)], [B(i,3), C(i,3)], ...
            colors{i}, 'LineWidth', 2);
    end
    
    % --- PLATFORM JOINTS (spheres) ---
    for i = 1:3
        [X, Y, Z] = sphere(16);
        radius = 0.007;
        X = X * radius + C(i,1);
        Y = Y * radius + C(i,2);
        Z = Z * radius + C(i,3);
        surf(ax, X, Y, Z, 'FaceColor', [0.9 0.7 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    end
    
    % --- MOVING PLATFORM ---
    patch(ax, 'XData', C([1:3 1],1), 'YData', C([1:3 1],2), 'ZData', C([1:3 1],3), ...
          'FaceColor', 'c', 'EdgeColor', 'b', 'LineWidth', 2, 'FaceAlpha', 0.6, 'DisplayName', 'Platform');
    
    % --- TCP ---
    plot3(ax, P(1), P(2), z, 'm*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'TCP');
    
    %% 4. Draw reference horizontal plane
    phi = linspace(0, 2*pi, 100);
    ref_circle_x = Rb * 1.5 * cos(phi);
    ref_circle_y = Rb * 1.5 * sin(phi);
    ref_circle_z = zeros(size(phi));
    plot3(ax, ref_circle_x, ref_circle_y, ref_circle_z, 'k:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    text(ax, Rb*1.5, 0, 0, 'Horizontal Ref (Z=0)', 'FontSize', 7, 'Color', 'k', 'HandleVisibility', 'off');
    
    %% 5. Finalize
    legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 9);
    
    % Set limits to show both upward and downward motion
    min_z = min(min(B(:,3)), -0.1);
    max_z = max([max(B(:,3)), z]) + 0.05;
    xlim(ax, [-0.35 0.35]);
    ylim(ax, [-0.35 0.35]);
    zlim(ax, [min_z, max_z]);
    
    hold off; 
    drawnow;
    
    %% 6. Status
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('                    DELTA ROBOT STATUS REPORT                    \n');
    fprintf('═══════════════════════════════════════════════════════════════\n\n');
    fprintf('  • Motors at EDGE MIDPOINTS of base triangle\n');
    fprintf('  • Base: Z = 0 (TOP)\n');
    fprintf('  • Platform: Z = %.3f m (BOTTOM)\n', z);
    fprintf('  • Angles measured from HORIZONTAL:\n');
    fprintf('      - Positive: INWARD + DOWNWARD\n');
    fprintf('      - Negative: OUTWARD + UPWARD\n\n');
    fprintf('  • Joint Angles:\n');
    for i = 1:3
        if theta(i) > 0
            dir_str = 'inward/downward';
        elseif theta(i) < 0
            dir_str = 'outward/upward';
        else
            dir_str = 'horizontal';
        end
        fprintf('      θ%d = %6.1f° (%s)\n', i, rad2deg(theta(i)), dir_str);
    end
    fprintf('\n═══════════════════════════════════════════════════════════════\n');
end