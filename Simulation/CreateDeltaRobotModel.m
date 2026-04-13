function robot = CreateDeltaRobotModel(params)
% CREATEDELTAROBOTMODEL Builds a rigidBodyTree proxy for a Delta robot.
%   The tree uses 3 prismatic joints to represent the Delta's translational DOFs.
%   Your analytical deltaFK/deltaIK drive the actual motion; this tree 
%   provides MATLAB Toolbox API compatibility (show, getTransform, etc.).

    robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 4);
    
    % Base frame (fixed)
    baseBody = rigidBody('base_link');
    addBody(robot, baseBody, 'base');
    
    % X-axis prismatic joint
    xBody = rigidBody('x_link');
    xJoint = rigidBodyJoint('j_x', 'prismatic');
    xJoint.JointAxis = [1 0 0];
    % --- FIX: Set limits here on the joint object ---
    xJoint.JointLimits = [params.workspace.x(1), params.workspace.x(2)]; 
    setFixedTransform(xJoint, trvec2tform([0 0 0]));
    xBody.Joint = xJoint;
    addBody(robot, xBody, 'base_link');
    
    % Y-axis prismatic joint
    yBody = rigidBody('y_link');
    yJoint = rigidBodyJoint('j_y', 'prismatic');
    yJoint.JointAxis = [0 1 0];
    % --- FIX: Set limits here on the joint object ---
    yJoint.JointLimits = [params.workspace.y(1), params.workspace.y(2)];
    setFixedTransform(yJoint, trvec2tform([0 0 0]));
    yBody.Joint = yJoint;
    addBody(robot, yBody, 'x_link');
    
    % Z-axis prismatic joint + tool frame
    zBody = rigidBody('z_link');
    zJoint = rigidBodyJoint('j_z', 'prismatic');
    zJoint.JointAxis = [0 0 1];
    % --- FIX: Set limits here on the joint object ---
    zJoint.JointLimits = [params.workspace.z(1), params.workspace.z(2)];
    setFixedTransform(zJoint, trvec2tform([0 0 0]));
    zBody.Joint = zJoint;
    addBody(robot, zBody, 'y_link');
    
    % Tool/End-Effector frame
    toolBody = rigidBody('tool');
    fixJoint = rigidBodyJoint('fix_tool', 'fixed');
    setFixedTransform(fixJoint, trvec2tform([0 0 0]));
    toolBody.Joint = fixJoint;
    addBody(robot, toolBody, 'z_link');
    
    fprintf('✅ rigidBodyTree created with 3 prismatic DOFs\n');
    fprintf('   X: [%.3f, %.3f] m | Y: [%.3f, %.3f] m | Z: [%.3f, %.3f] m\n', ...
        params.workspace.x(1), params.workspace.x(2), ...
        params.workspace.y(1), params.workspace.y(2), ...
        params.workspace.z(1), params.workspace.z(2));
end