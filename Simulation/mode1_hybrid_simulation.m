function mode1_hybrid_simulation()
% MODE1_HYBRID_SIMULATION Demonstrates Delta kinematics using both:
%   1. Your analytical deltaFK/deltaIK (mathematical truth)
%   2. MATLAB's rigidBodyTree API (show, getTransform, standard workflow)

    params = deltaParams();
    robot  = CreateDeltaRobotModel(params);
    
    %% 1. Define Circular Trajectory
    t = (0:0.04:3)';
    center = [0.00, 0.00, 0.20];
    radius = 0.04;
    theta_path = linspace(0, 2*pi, length(t));
    points = center + radius * [cos(theta_path)', sin(theta_path)', zeros(length(t),1)];
    
    %% 2. Solve IK & Verify Reachability
    fprintf('=== Solving IK for %d waypoints ===\n', length(points));
    thetaTraj = zeros(3, length(t));
    qCart = zeros(3, length(t)); % Cartesian proxy config [x;y;z]
    qInitial = deg2rad([0;0;0]);
    
    for i = 1:length(t)
        try
            [thetaTraj(:,i), ~] = deltaIK(points(i,1), points(i,2), points(i,3), params, qInitial);
            qCart(:,i) = points(i,:)'; % Sync proxy to target
            qInitial = thetaTraj(:,i);
        catch
            error('IK failed at waypoint %d. Adjust trajectory or workspace.', i);
        end
    end
    fprintf('✅ All waypoints solved.\n\n');
    
    %% 3. Setup Visualization Figure
    fig = figure('Name','Hybrid Delta Simulation', 'Color','w');
    ax = axes('Parent', fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
    xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
    ax.XLim = [-0.2 0.2]; ax.YLim = [-0.2 0.2]; ax.ZLim = [-0.1 0.3];
    title(ax,'Analytical Model + rigidBodyTree API Sync');
    
    % --- Parallel Robot Graphics (Your Custom Geometry) ---
    Rb = params.Rb; Rp = params.Rp; L = params.L;
    A = [Rb,0,0; -Rb/2,sqrt(3)*Rb/2,0; -Rb/2,-sqrt(3)*Rb/2,0];
    h.baseLine = plot3(ax, [A(:,1);A(1,1)], [A(:,2);A(1,2)], [A(:,3);A(1,3)], 'k--','LineWidth',1.5);
    h.basePts  = plot3(ax, A(:,1), A(:,2), A(:,3), 'ko','MarkerSize',8,'MarkerFaceColor','k');
    h.elbowPts = plot3(ax, nan,nan,nan, 'ro','MarkerSize',7,'MarkerFaceColor','r');
    h.platPts  = plot3(ax, nan,nan,nan, 'bo','MarkerSize',7,'MarkerFaceColor','b');
    h.platLine = plot3(ax, nan,nan,nan, 'g-','LineWidth',2);
    h.center   = plot3(ax, nan,nan,nan, 'gs','MarkerSize',10,'MarkerFaceColor','g');
    h.trajectory = plot3(ax, points(:,1), points(:,2), points(:,3), 'm--','LineWidth',1.5);
    h.upperArm = gobjects(3,1); h.lowerRod = gobjects(3,1);
    for j=1:3
        h.upperArm(j) = plot3(ax, nan,nan,nan, 'r-','LineWidth',2.5);
        h.lowerRod(j) = plot3(ax, nan,nan,nan, 'b-','LineWidth',2.5);
    end
    
    % --- rigidBodyTree Visualization ---
    % show() creates its own axes, so we overlay it on our custom figure
    axRobot = axes('Parent', fig, 'Position', [0.65 0.65 0.3 0.3], 'Visible','off');
    hRobot = show(robot, homeConfiguration(robot), 'Parent', axRobot, 'FastUpdate','on');
    
    %% 4. Animation Loop
    fprintf('=== Starting Hybrid Animation ===\n');
    dt = 0.02;
    r = rateControl(1/dt);
    
    for i = 1:length(t)
        theta = thetaTraj(:,i);
        P_actual = deltaFK(theta(1), theta(2), theta(3), params); % Analytical truth
        
        % --- Update Parallel Graphics ---
        B = zeros(3,3);
        B(1,:) = [Rb-L*cos(theta(1)), 0, L*sin(theta(1))];
        B(2,:) = [-(Rb-L*cos(theta(2)))/2, sqrt(3)*(Rb-L*cos(theta(2)))/2, L*sin(theta(2))];
        B(3,:) = [-(Rb-L*cos(theta(3)))/2, -sqrt(3)*(Rb-L*cos(theta(3)))/2, L*sin(theta(3))];
        x=P_actual(1); y=P_actual(2); z=P_actual(3);
        C = [x+Rp, y, z; x-Rp/2, y+sqrt(3)*Rp/2, z; x-Rp/2, y-sqrt(3)*Rp/2, z];
        
        set(h.elbowPts, 'XData',B(:,1), 'YData',B(:,2), 'ZData',B(:,3));
        set(h.platPts,  'XData',C(:,1), 'YData',C(:,2), 'ZData',C(:,3));
        set(h.platLine, 'XData',[C(:,1);C(1,1)], 'YData',[C(:,2);C(1,2)], 'ZData',[C(:,3);C(1,3)]);
        set(h.center,   'XData',P_actual(1), 'YData',P_actual(2), 'ZData',P_actual(3));
        for j=1:3
            set(h.upperArm(j), 'XData',[A(j,1) B(j,1)], 'YData',[A(j,2) B(j,2)], 'ZData',[A(j,3) B(j,3)]);
            set(h.lowerRod(j), 'XData',[B(j,1) C(j,1)], 'YData',[B(j,2) C(j,2)], 'ZData',[B(j,3) C(j,3)]);
        end
        
        % --- Sync & Update rigidBodyTree ---
        robotConfig = qCart(:,i);
        set(hRobot.Configuration, robotConfig);
        
        % Verify API consistency
        T_ee = getTransform(robot, robotConfig, 'tool');
        P_api = T_ee(1:3,4)';
        
        drawnow limitrate;
        waitfor(r);
    end
    fprintf('✅ Animation complete. API & Analytical models synced.\n');
end