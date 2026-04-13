function [P, info] = deltaFK(theta1, theta2, theta3, params)
% deltaFK  Forward kinematics of a 3-DOF delta robot.
%
%   [P, info] = deltaFK(theta1, theta2, theta3, params)
%
%   INPUTS
%       theta1, theta2, theta3 : Joint angles [rad]
%       params                 : Structure returned by deltaParams()
%
%   OUTPUTS
%       P    : 3x1 platform-center position [x; y; z] in meters
%       info : Diagnostic structure containing:
%              - determinant / discriminant information
%              - all candidate z roots
%              - selected root and reason
%              - loop-closure residuals
%              - workspace check result
%
%   METHOD
%       1) Build the three elbow points B1, B2, B3
%       2) Subtract the three sphere equations to obtain x(z), y(z)
%       3) Solve the quadratic in z
%       4) Evaluate each real root for workspace and residual consistency
%       5) Select the physically consistent root according to the configured
%          policy, then verify loop closure

arguments
    theta1 (1,1) double
    theta2 (1,1) double
    theta3 (1,1) double
    params (1,1) struct
end

theta = [theta1; theta2; theta3];
tol = params.tol;

if any(theta < params.thetaLimits(:,1) - tol | theta > params.thetaLimits(:,2) + tol)
    error('deltaFK:JointLimitViolation', ...
        'One or more input joint angles violate the specified limits.');
end

Rb = params.Rb;
Rp = params.Rp;
L  = params.L;
l  = params.l;

info = struct();
info.inputTheta = theta;

Delta = Rb - Rp;

alpha1 = Delta - L*cos(theta1);
alpha2 = Delta - L*cos(theta2);
alpha3 = Delta - L*cos(theta3);

beta1 = L*sin(theta1);
beta2 = L*sin(theta2);
beta3 = L*sin(theta3);

D1 = [ alpha1;                 0;                 beta1];
D2 = [-alpha2/2;  sqrt(3)*alpha2/2;               beta2];
D3 = [-alpha3/2; -sqrt(3)*alpha3/2;               beta3];
info.D1 = D1;
info.D2 = D2;
info.D3 = D3;

a11 = -alpha2/2 - alpha1;
a12 =  sqrt(3)*alpha2/2;
a13 =  beta2 - beta1;

a21 = -alpha3/2 - alpha1;
a22 = -sqrt(3)*alpha3/2;
a23 =  beta3 - beta1;

r1 = 0.5*(alpha2^2 + beta2^2 - alpha1^2 - beta1^2);
r2 = 0.5*(alpha3^2 + beta3^2 - alpha1^2 - beta1^2);

detA = a11*a22 - a12*a21;
info.detA = detA;

if abs(detA) < params.detTol
    error('deltaFK:SingularConfiguration', ...
        'The FK 2x2 system is singular or nearly singular (detA = %.3e).', detA);
end

px = (r1*a22 - a12*r2) / detA;
qx = (-a13*a22 + a12*a23) / detA;

py = (a11*r2 - a21*r1) / detA;
qy = (-a11*a23 + a21*a13) / detA;

info.px = px;
info.qx = qx;
info.py = py;
info.qy = qy;

Az = qx^2 + qy^2 + 1;
Bz = 2*(qx*(px - alpha1) + qy*py - beta1);
Cz = (px - alpha1)^2 + py^2 + beta1^2 - l^2;

disc = Bz^2 - 4*Az*Cz;
info.Az = Az;
info.Bz = Bz;
info.Cz = Cz;
info.discriminant = disc;

if disc < -params.discTol
    error('deltaFK:ComplexZ', ...
        'The FK quadratic discriminant is negative (disc = %.3e).', disc);
end

disc = max(disc, 0);
zRoots = [(-Bz + sqrt(disc)) / (2*Az);
          (-Bz - sqrt(disc)) / (2*Az)];
info.zRoots = zRoots;

[rootCandidates, feasibleMask] = evaluateZroots(zRoots, px, qx, py, qy, theta, params);
info.rootCandidates = rootCandidates;
info.numFeasibleRoots = nnz(feasibleMask);

if ~any(feasibleMask)
    error('deltaFK:NoFeasibleRoot', ...
        'No FK root satisfied both workspace and residual checks.');
end

selectedIdx = selectRootIndex(rootCandidates, feasibleMask, params);
selected = rootCandidates(selectedIdx);

P = selected.P;
info.selectedRootIndex = selectedIdx;
info.selectedRoot = selected.z;
info.selectedRootReason = selected.reason;
info.position = P;
info.workspaceOK = selected.workspaceOK;
info.loopResiduals = selected.loopResiduals;
info.maxResidual = selected.maxResidual;

if info.maxResidual > params.residualTol
    error('deltaFK:ResidualTooLarge', ...
        'FK residual check failed. Max residual = %.3e', info.maxResidual);
end

end

function [rootCandidates, feasibleMask] = evaluateZroots(zRoots, px, qx, py, qy, theta, params)
rootCandidates = repmat(struct( ...
    'z', nan, ...
    'P', nan(3,1), ...
    'workspaceOK', false, ...
    'loopResiduals', nan(3,1), ...
    'maxResidual', nan, ...
    'reason', ""), numel(zRoots), 1);

feasibleMask = false(numel(zRoots),1);

for i = 1:numel(zRoots)
    z = zRoots(i);
    x = px + qx*z;
    y = py + qy*z;
    P = [x; y; z];

    res = localLoopResiduals(P, theta, params);
    maxResidual = max(abs(res));
    workspaceOK = isInsideWorkspace(P, params);

    rootCandidates(i).z = z;
    rootCandidates(i).P = P;
    rootCandidates(i).workspaceOK = workspaceOK;
    rootCandidates(i).loopResiduals = res;
    rootCandidates(i).maxResidual = maxResidual;
    rootCandidates(i).reason = rootReason(workspaceOK, maxResidual, params);

    feasibleMask(i) = workspaceOK && (maxResidual <= params.rootResidualTol);
end
end

function idx = selectRootIndex(rootCandidates, feasibleMask, params)
feasibleIdx = find(feasibleMask);
feasibleZ = arrayfun(@(s) s.z, rootCandidates(feasibleIdx));

switch lower(params.fkRootPolicy)
    case 'smallestpositive'
        posMask = feasibleZ >= params.workspace.z(1) - params.tol;
        if any(posMask)
            posIdx = feasibleIdx(posMask);
            posZ   = feasibleZ(posMask);
            [~, rel] = min(posZ);
            idx = posIdx(rel);
        else
            [~, rel] = min(feasibleZ);
            idx = feasibleIdx(rel);
        end
    case 'largestpositive'
        posMask = feasibleZ >= params.workspace.z(1) - params.tol;
        if any(posMask)
            posIdx = feasibleIdx(posMask);
            posZ   = feasibleZ(posMask);
            [~, rel] = max(posZ);
            idx = posIdx(rel);
        else
            [~, rel] = max(feasibleZ);
            idx = feasibleIdx(rel);
        end
    case 'closesttomidz'
        zMid = mean(params.workspace.z);
        [~, rel] = min(abs(feasibleZ - zMid));
        idx = feasibleIdx(rel);
    otherwise
        error('deltaFK:UnknownRootPolicy', ...
            'Unknown FK root policy: %s', params.fkRootPolicy);
end
end

function txt = rootReason(workspaceOK, maxResidual, params)
if workspaceOK && (maxResidual <= params.rootResidualTol)
    txt = "Accepted root";
elseif ~workspaceOK && (maxResidual <= params.rootResidualTol)
    txt = "Rejected: outside workspace";
elseif workspaceOK && (maxResidual > params.rootResidualTol)
    txt = "Rejected: residual too large";
else
    txt = "Rejected: outside workspace and residual too large";
end
end

function tf = isInsideWorkspace(P, params)
x = P(1); y = P(2); z = P(3);
tol = params.tol;

tf = (x >= params.workspace.x(1) - tol) && (x <= params.workspace.x(2) + tol) && ...
     (y >= params.workspace.y(1) - tol) && (y <= params.workspace.y(2) + tol) && ...
     (z >= params.workspace.z(1) - tol) && (z <= params.workspace.z(2) + tol);
end

function res = localLoopResiduals(P, theta, params)
Rb = params.Rb; Rp = params.Rp; L = params.L; l = params.l;
x = P(1); y = P(2); z = P(3);

C1 = [x + Rp;               y;                         z];
C2 = [x - Rp/2;             y + sqrt(3)*Rp/2;         z];
C3 = [x - Rp/2;             y - sqrt(3)*Rp/2;         z];

B1 = [Rb - L*cos(theta(1)); 0;                                      L*sin(theta(1))];
B2 = [-(Rb - L*cos(theta(2)))/2;  sqrt(3)*(Rb - L*cos(theta(2)))/2; L*sin(theta(2))];
B3 = [-(Rb - L*cos(theta(3)))/2; -sqrt(3)*(Rb - L*cos(theta(3)))/2; L*sin(theta(3))];

res = [norm(C1 - B1)^2 - l^2;
       norm(C2 - B2)^2 - l^2;
       norm(C3 - B3)^2 - l^2];
end
