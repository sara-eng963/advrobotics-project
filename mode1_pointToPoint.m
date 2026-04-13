function mode1_pointToPoint()
% MODE1_POINTTOPOINT  Animates the Delta robot moving between 5 target positions.
%   Uses the correct visualization model with:
%   - Motors at edge midpoints of base triangle
%   - Angles measured from horizontal
%   - Base at TOP, Platform at BOTTOM (Z positive DOWNWARD)

    params = deltaParams();
    
    %% 1. DEFINE WAYPOINTS (x, y, z) in meters
    % Z positive DOWNWARD (consistent with deltaParams)
    waypoints = [
    0.00,  0.00,  0.22;   % Center start
    0.05,  0.00,  0.22;   % Move +X
    0.05,  0.05,  0.22;   % Move +Y
    0.00,  0.05,  0.22;   % Move -X
    0.00,  0.00,  0.22;   % Return to center
];
    
    %% 2. COMPUTE INVERSE KINEMATICS FOR EACH WAYPOINT
    nWP = size(waypoints, 1);
    thetaWP = zeros(3, nWP);
    
    fprintf('========================================\n');
    fprintf('Mode 1: Point-to-Point Trajectory\n');
    fprintf('========================================\n');
    fprintf('Coordinate System: Z positive DOWNWARD\n');
    fprintf('Base at TOP (Z=0), Platform at BOTTOM (Z>0)\n');
    fprintf('Checking reachability for %d waypoints...\n', nWP);
    
    for i = 1:nWP
        p = waypoints(i,:)';
        try
            [thetaWP(:,i), ikInfo] = deltaIK(p(1), p(2), p(3), params);
            fprintf('  WP%d ✅ Reachable. θ = [%6.1f°, %6.1f°, %6.1f°]\n', ...
                i, rad2deg(thetaWP(1,i)), rad2deg(thetaWP(2,i)), rad2deg(thetaWP(3,i)));
        catch ME
            fprintf('  WP%d ❌ UNREACHABLE: %s\n', i, ME.message);
            error('Aborting: Waypoint %d is outside valid workspace.', i);
        end
    end
    
    %% 3. INTERPOLATE JOINT ANGLES BETWEEN WAYPOINTS
    stepsPerSegment = 50;
    
    nSteps = (nWP - 1) * stepsPerSegment;
    thetaTraj = zeros(3, nSteps);
    idx = 1;
    
    for seg = 1:nWP-1
        startAng = thetaWP(:, seg);
        endAng   = thetaWP(:, seg+1);
        
        for s = 1:stepsPerSegment
            alpha = s / stepsPerSegment;
            thetaTraj(:, idx) = startAng + alpha * (endAng - startAng);
            idx = idx + 1;
        end
    end
    
    %% 4. SETUP ANIMATION FIGURE
    fig = figure('Name', 'Mode 1: Point-to-Point Animation', 'Color', 'w', ...
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
    title(ax, 'Mode 1: Point-to-Point Trajectory - Delta Robot');
    
    % Set fixed axis limits
    ax.XLim = [-0.25 0.25];
    ax.YLim = [-0.25 0.25];
    ax.ZLim = [-0.05 0.35];
    
    %% 5. PRE-COMPUTE GEOMETRY
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
    
    %% 6. CREATE GRAPHICS HANDLES
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
    
    % Elbow joints (individual for colors)
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
    
    % Platform triangle (at BOTTOM)
    h.platform = patch(ax, 'XData', nan, 'YData', nan, 'ZData', nan, ...
                       'FaceColor', 'c', 'EdgeColor', 'b', 'LineWidth', 2, ...
                       'FaceAlpha', 0.6, 'DisplayName', 'Platform (BOTTOM)');
    
    % Platform joints
    h.platformJoints = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 8, ...
                             'MarkerFaceColor', [0.9 0.7 0.2], 'MarkerEdgeColor', 'k', ...
                             'HandleVisibility', 'off');
    
    % TCP
    h.tcp = plot3(ax, nan, nan, nan, 'm*', 'MarkerSize', 12, 'LineWidth', 2, ...
                  'DisplayName', 'TCP');
    
    % Trajectory path
    h.trajectory = plot3(ax, waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
                         'm--', 'LineWidth', 1.5, 'DisplayName', 'Target Path');
    
    legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 8);
    
    %% 7. ANIMATION LOOP
    fprintf('\nStarting animation (%d frames)...\n', nSteps);
    tic;
    
    for k = 1:nSteps
        theta = thetaTraj(:, k);
        
        % Elbow positions (Z positive DOWNWARD)
        B = zeros(3,3);
        for i = 1:3
            % Direction from motor to center
            to_center = -[A(i,1), A(i,2)] / norm(A(i,1:2));
            
            % Direction based on angle sign
            if theta(i) >= 0
                direction = to_center;   % Point INWARD
            else
                direction = -to_center;  % Point OUTWARD
            end
            
            % Horizontal distance
            dist = L * cos(abs(theta(i)));
            
            % Elbow position
            B(i,1) = A(i,1) + direction(1) * dist;
            B(i,2) = A(i,2) + direction(2) * dist;
            % Z: positive = DOWNWARD (since ZDir reverse makes it look correct)
            B(i,3) = L * abs(sin(theta(i)));  % ALWAYS positive (downward)
        end
        
        % Platform position (FK returns Z positive DOWNWARD)
        P = deltaFK(theta(1), theta(2), theta(3), params);
        x = P(1); y = P(2); z = abs(P(3));
        
        % Platform joints
        C = zeros(3,3);
        for i = 1:3
            angle = (i-1) * 120 * pi/180;
            C(i,:) = [x + Rp * cos(angle), y + Rp * sin(angle), z];
        end
        
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
        set(h.tcp, 'XData', P(1), 'YData', P(2), 'ZData', z);
        
        drawnow limitrate;
        pause(0.02);
    end
    
    elapsed = toc;
    fprintf('\n✅ Animation complete! (%d frames in %.2f seconds, %.1f FPS)\n', ...
            nSteps, elapsed, nSteps/elapsed);
    fprintf('========================================\n');
end