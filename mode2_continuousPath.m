function mode2_continuousPath()
% MODE2_CONTINUOUSPATH  Animates the Delta robot following a smooth continuous path.
%   Automatically detects workspace limits and adapts path size.

    params = deltaParams();
    
    %% 1. FIRST, FIND THE REACHABLE WORKSPACE
    fprintf('========================================\n');
    fprintf('Mode 2: Continuous Path Trajectory\n');
    fprintf('========================================\n');
    fprintf('Detecting robot workspace...\n');
    
    % Test reachable Z range at center
    z_center = 0.20;  % Start with 20cm
    test_z = [0.18, 0.20, 0.22, 0.24, 0.26];
    reachable_z = [];
    for z = test_z
        try
            [theta, ~] = deltaIK(0, 0, z, params);
            reachable_z = [reachable_z, z];
            fprintf('  ✓ Center at Z=%.3f m is reachable\n', z);
        catch
            fprintf('  ✗ Center at Z=%.3f m is NOT reachable\n', z);
        end
    end
    
    if isempty(reachable_z)
        error('No reachable Z found at center! Check your robot dimensions.');
    end
    
    % Use the middle of reachable Z range
    safe_z = reachable_z(ceil(end/2));
    fprintf('\nUsing safe Z height: %.3f m\n', safe_z);
    
    % Test reachable XY range at safe Z
    test_radii = [0.01, 0.02, 0.03, 0.04];
    max_radius = 0;
    for r = test_radii
        try
            [theta, ~] = deltaIK(r, 0, safe_z, params);
            max_radius = r;
            fprintf('  ✓ Radius %.3f m is reachable\n', r);
        catch
            fprintf('  ✗ Radius %.3f m is NOT reachable\n', r);
            break;
        end
    end
    
    % Use 80% of max radius for safety
    safe_radius = max_radius * 0.8;
    fprintf('\nUsing safe radius: %.3f m\n', safe_radius);
    
    %% 2. SELECT PATH TYPE
    pathType = 'circle';  % Options: 'circle', 'line', 'figure8', 'spiral'
    
    %% 3. GENERATE WAYPOINTS (using safe dimensions)
    nPoints = 80;  % Number of points along the path
    
    switch pathType
        case 'circle'
            % Circle path
            radius = safe_radius;
            center = [0, 0, safe_z];
            angles = linspace(0, 2*pi, nPoints);
            waypoints = zeros(nPoints, 3);
            for i = 1:nPoints
                waypoints(i,:) = [center(1) + radius * cos(angles(i)), ...
                                  center(2) + radius * sin(angles(i)), ...
                                  center(3)];
            end
            fprintf('\nCircle path: radius = %.3f m at Z = %.3f m\n', radius, safe_z);
            
        case 'line'
            % Line path back and forth
            x_range = [-safe_radius, safe_radius];
            y_fixed = 0;
            z_fixed = safe_z;
            t = linspace(0, 1, nPoints);
            x = x_range(1) + (x_range(2) - x_range(1)) * sin(2*pi*t);
            waypoints = [x', y_fixed * ones(nPoints,1), z_fixed * ones(nPoints,1)];
            fprintf('\nLine path: X from %.2f to %.2f m at Y=0, Z=%.2f m\n', x_range(1), x_range(2), z_fixed);
            
        case 'figure8'
            % Figure-8 pattern
            scale = safe_radius;
            center_z = safe_z;
            t = linspace(0, 2*pi, nPoints);
            x = scale * sin(t);
            y = scale * sin(t).*cos(t);
            waypoints = [x', y', center_z * ones(nPoints,1)];
            fprintf('\nFigure-8 path: scale = %.3f m at Z = %.3f m\n', scale, center_z);
            
        case 'spiral'
            % Spiral pattern
            max_radius = safe_radius;
            turns = 1;
            t = linspace(0, turns*2*pi, nPoints);
            radius = max_radius * t / (turns*2*pi);
            x = radius .* cos(t);
            y = radius .* sin(t);
            z = safe_z + 0.005 * sin(t);  % Small Z variation
            waypoints = [x', y', z'];
            fprintf('\nSpiral path: max radius = %.3f m, Z from %.3f to %.3f m\n', max_radius, safe_z-0.005, safe_z+0.005);
    end
    
    %% 4. COMPUTE INVERSE KINEMATICS FOR ALL WAYPOINTS
    nWP = size(waypoints, 1);
    thetaWP = zeros(3, nWP);
    
    fprintf('\nComputing IK for %d waypoints...\n', nWP);
    
    reachableCount = 0;
    for i = 1:nWP
        p = waypoints(i,:)';
        try
            [thetaWP(:,i), ~] = deltaIK(p(1), p(2), p(3), params);
            reachableCount = reachableCount + 1;
        catch
            % If a point is unreachable, interpolate between neighbors
            if i > 1 && i < nWP
                thetaWP(:,i) = (thetaWP(:,i-1) + thetaWP(:,i+1)) / 2;
            elseif i > 1
                thetaWP(:,i) = thetaWP(:,i-1);
            else
                thetaWP(:,i) = thetaWP(:,i+1);
            end
        end
    end
    
    fprintf('Success rate: %d/%d points (%.1f%%)\n', reachableCount, nWP, 100*reachableCount/nWP);
    
    %% 5. SETUP ANIMATION FIGURE
    fig = figure('Name', 'Mode 2: Continuous Path Animation', 'Color', 'w', ...
                 'DoubleBuffer', 'on', 'Renderer', 'opengl');
    set(fig, 'Position', [100, 100, 1000, 800]);
    
    ax = axes('Parent', fig);
    hold(ax, 'on'); 
    grid(ax, 'on'); 
    axis(ax, 'equal'); 
    box(ax, 'on');
    
    % CRITICAL: ZDir reverse makes Z positive go DOWN on screen
    set(ax, 'ZDir', 'reverse');
    view(ax, 3);
    
    xlabel(ax, 'X [m]'); 
    ylabel(ax, 'Y [m]'); 
    zlabel(ax, 'Z [m] (DOWNWARD)');
    title(ax, sprintf('Mode 2: %s Path - Delta Robot (Radius: %.1f cm)', ...
          upper(pathType), safe_radius*100));
    
    % Set fixed axis limits
    ax.XLim = [-0.25 0.25];
    ax.YLim = [-0.25 0.25];
    ax.ZLim = [-0.05 0.35];
    
    %% 6. PRE-COMPUTE GEOMETRY
    Rb = params.Rb; Rp = params.Rp; L = params.L;
    
    % Triangle vertices (base triangle at Z=0, TOP)
    vertices = zeros(3,3);
    for i = 1:3
        angle = (i-1) * 120 * pi/180;
        vertices(i,:) = [Rb * cos(angle), Rb * sin(angle), 0];
    end
    
    % Motor positions at edge midpoints (at TOP, Z=0)
    A = zeros(3,3);
    for i = 1:3
        next = mod(i, 3) + 1;
        A(i,:) = (vertices(i,:) + vertices(next,:)) / 2;
    end
    
    %% 7. CREATE GRAPHICS HANDLES
    colors = {'r', 'g', 'b'};
    
    % Base triangle (filled, at TOP)
    h.baseFill = fill3(ax, vertices(:,1), vertices(:,2), vertices(:,3), ...
                       [0.3 0.3 0.3], 'FaceAlpha', 0.3, 'EdgeColor', 'k', ...
                       'LineWidth', 2, 'DisplayName', 'Base (TOP)');
    
    % Motors
    h.motors = plot3(ax, A(:,1), A(:,2), A(:,3), 'ko', 'MarkerSize', 12, ...
                     'MarkerFaceColor', 'k', 'DisplayName', 'Motors');
    
    % Upper arms
    h.upperArms = gobjects(3,1);
    for i = 1:3
        h.upperArms(i) = plot3(ax, nan, nan, nan, colors{i}, 'LineWidth', 4, ...
                               'DisplayName', sprintf('Arm %d', i));
    end
    
    % Elbow joints
    h.elbows = gobjects(3,1);
    for i = 1:3
        h.elbows(i) = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 10, ...
                            'MarkerFaceColor', 'w', 'MarkerEdgeColor', colors{i}, ...
                            'LineWidth', 2, 'HandleVisibility', 'off');
    end

    
    % Lower rods
    h.lowerRods = gobjects(3,1);
    for i = 1:3
        h.lowerRods(i) = plot3(ax, nan, nan, nan, colors{i}, 'LineWidth', 2, ...
                               'HandleVisibility', 'off');
    end
    
    % Platform triangle
    h.platform = patch(ax, 'XData', nan, 'YData', nan, 'ZData', nan, ...
                       'FaceColor', 'c', 'EdgeColor', 'b', 'LineWidth', 2, ...
                       'FaceAlpha', 0.6, 'DisplayName', 'Platform');
    
    % Platform joints
    h.platformJoints = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 8, ...
                             'MarkerFaceColor', [0.9 0.7 0.2], 'MarkerEdgeColor', 'k', ...
                             'HandleVisibility', 'off');
    
    % TCP
    h.tcp = plot3(ax, nan, nan, nan, 'm*', 'MarkerSize', 12, 'LineWidth', 2, ...
                  'DisplayName', 'TCP');
    
    % Full trajectory path
    h.trajectory = plot3(ax, waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
                         'm-', 'LineWidth', 1.5, 'DisplayName', 'Target Path');
    
    % Moving trace
    h.trace = plot3(ax, nan, nan, nan, 'c.', 'MarkerSize', 3, 'HandleVisibility', 'off');
    
    legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 8);
    
    %% 8. ANIMATION LOOP
    fprintf('\nStarting animation (%d frames)...\n', nWP);
    tic;
    
    % Store trace points
    tracePoints = [];
    
    for k = 1:nWP
        theta = thetaWP(:, k);
        
        % Elbow positions
        B = zeros(3,3);
        for i = 1:3
            to_center = -[A(i,1), A(i,2)] / norm(A(i,1:2));
            
            if theta(i) >= 0
                direction = to_center;
            else
                direction = -to_center;
            end
            
            dist = L * cos(abs(theta(i)));
            
            B(i,1) = A(i,1) + direction(1) * dist;
            B(i,2) = A(i,2) + direction(2) * dist;
            B(i,3) = L * abs(sin(theta(i)));
        end
        
        % Platform position
        P = deltaFK(theta(1), theta(2), theta(3), params);
        x = P(1); y = P(2); z = abs(P(3));
        
        % Platform joints
        C = zeros(3,3);
        for i = 1:3
            angle = (i-1) * 120 * pi/180;
            C(i,:) = [x + Rp * cos(angle), y + Rp * sin(angle), z];
        end
        
        % Update trace
        tracePoints = [tracePoints; x, y, z];
        if size(tracePoints, 1) > 200
            tracePoints(1,:) = [];
        end
        set(h.trace, 'XData', tracePoints(:,1), 'YData', tracePoints(:,2), 'ZData', tracePoints(:,3));
        
        % Update graphics
        for i = 1:3
            set(h.upperArms(i), 'XData', [A(i,1), B(i,1)], ...
                               'YData', [A(i,2), B(i,2)], ...
                               'ZData', [A(i,3), B(i,3)]);
            
            set(h.elbows(i), 'XData', B(i,1), 'YData', B(i,2), 'ZData', B(i,3));
            
            set(h.lowerRods(i), 'XData', [B(i,1), C(i,1)], ...
                               'YData', [B(i,2), C(i,2)], ...
                               'ZData', [B(i,3), C(i,3)]);
        end
        
        set(h.platform, 'XData', [C(:,1); C(1,1)], ...
                       'YData', [C(:,2); C(1,2)], ...
                       'ZData', [C(:,3); C(1,3)]);
        
        set(h.platformJoints, 'XData', C(:,1), 'YData', C(:,2), 'ZData', C(:,3));
        set(h.tcp, 'XData', x, 'YData', y, 'ZData', z);
        
        drawnow limitrate;
        pause(0.02);
    end
    
    elapsed = toc;
    fprintf('\n✅ Animation complete! (%d frames in %.2f seconds, %.1f FPS)\n', ...
            nWP, elapsed, nWP/elapsed);
    fprintf('========================================\n');
end