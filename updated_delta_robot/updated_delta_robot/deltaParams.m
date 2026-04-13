function params = deltaParams()
% deltaParams  Geometry, limits, numerical tolerances, and validation
% settings for the 3-DOF delta robot MATLAB model.
%
%   params = deltaParams()
%
%   IMPORTANT:
%   Replace the geometric placeholder values below with the final values
%   extracted from the CAD model used by the team.
%
%   Coordinate convention used by this model:
%       - Origin at the center of the fixed base triangle
%       - z-axis points downward
%       - Moving platform remains parallel to the fixed base
%
%   Geometric parameters:
%       Rb : base radius      (center of fixed base -> motor axis)
%       Rp : platform radius  (platform center -> platform joint)
%       L  : upper arm length
%       l  : lower rod length (effective spherical-joint to spherical-joint)

params = struct();

%% Geometry (replace with final CAD values)
params.Rb = 0.120;   % [m]
params.Rp = 0.035;   % [m]
params.L  = 0.10;   % [m]
params.l  = 0.30;   % [m]

%% Joint limits (rows correspond to theta1, theta2, theta3)
params.thetaLimits = deg2rad([-70  70;    % theta1 [deg]
                              -70  70;    % theta2 [deg]
                              -70  70]);  % theta3 [deg]

%% Cartesian workspace limits (software limits only)
params.workspace.x = [-0.150, 0.150];   % [m]
params.workspace.y = [-0.150, 0.150];   % [m]
params.workspace.z = [ 0.050, 0.300];   % [m], positive downward

%% Numerical tolerances
params.tol              = 1e-9;
params.detTol           = 1e-10;
params.discTol          = 1e-10;
params.residualTol      = 1e-6;
params.branchMatchTol   = 1e-3;   % [rad] same-branch recovery tolerance
params.duplicateTol     = 1e-8;   % [rad] duplicate-branch rejection tolerance
params.rootResidualTol  = 1e-6;   % [m^2] residual threshold for FK root acceptance

%% Branch handling
% Used when deltaIK is called without a reference posture.
params.defaultBranchPreference = [1; 1; 1];

%% FK root selection policy
% Supported:
%   'smallestPositive' : physically common when z is defined positive downward
%   'largestPositive'  : choose deeper positive root
%   'closestToMidZ'    : choose root nearest the middle of the software z-range
params.fkRootPolicy = 'smallestPositive';

%% Validation configuration for Milestone 1 - Point 4
params.validation.numCartesianTests = 30;
params.validation.numJointTests     = 30;
params.validation.randomSeed        = 7;
params.validation.defaultPoint      = [0.020; -0.015; 0.180];
params.validation.defaultThetaDeg   = [-40; -42; -41];

end
