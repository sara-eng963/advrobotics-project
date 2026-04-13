function [solutions, info] = deltaIKAllBranches(x, y, z, params)
% deltaIKAllBranches  Enumerate all valid IK branches for the delta robot.
%
%   [solutions, info] = deltaIKAllBranches(x, y, z, params)
%
%   INPUTS
%       x, y, z : Desired platform-center coordinates [m]
%       params  : Structure returned by deltaParams()
%
%   OUTPUTS
%       solutions : 3xN matrix. Each column is one valid and unique IK
%                   branch [rad].
%       info      : Diagnostic structure containing all raw candidates,
%                   rejection reasons, residual checks, and deduplication
%                   results.
%
%   METHOD
%       Each leg contributes two algebraic solutions, therefore up to 2^3 = 8
%       candidate angle triplets are generated. Every candidate is checked for:
%           1) software workspace consistency
%           2) reachability of each leg
%           3) joint limits
%           4) loop-closure residual tolerance
%           5) duplicate-branch rejection
%
%   This function only enumerates valid branches. Final branch selection is
%   handled by deltaIK().

arguments
    x (1,1) double
    y (1,1) double
    z (1,1) double
    params (1,1) struct
end

tol = params.tol;
commandedPoint = [x; y; z];

info = struct();
info.commandedPoint = commandedPoint;
info.workspacePassed = false;
info.candidates = repmat(struct( ...
    'branchSigns', nan(3,1), ...
    'theta', nan(3,1), ...
    'valid', false, ...
    'reason', "", ...
    'maxResidual', nan, ...
    'legData', []), 8, 1);
info.uniqueBranchMap = nan(1,8);

%% Workspace check
if ~isInsideWorkspace(commandedPoint, params)
    error('deltaIKAllBranches:WorkspaceViolation', ...
        'Requested point [%.6f, %.6f, %.6f] m violates the software workspace.', x, y, z);
end
info.workspacePassed = true;

%% Generate all 8 sign combinations
branchTable = [ -1 -1 -1;
                -1 -1  1;
                -1  1 -1;
                -1  1  1;
                 1 -1 -1;
                 1 -1  1;
                 1  1 -1;
                 1  1  1 ]';

solutions = zeros(3,0);
validCount = 0;

for k = 1:size(branchTable,2)
    branchSigns = branchTable(:,k);

    try
        [thetaCandidate, legData] = localComputeCandidate(x, y, z, params, branchSigns);
        thetaCandidate = wrapToPiLocal(thetaCandidate);

        limits = params.thetaLimits;
        if any(thetaCandidate < limits(:,1) - tol | thetaCandidate > limits(:,2) + tol)
            info.candidates(k).branchSigns = branchSigns;
            info.candidates(k).theta = thetaCandidate;
            info.candidates(k).valid = false;
            info.candidates(k).reason = "Rejected: joint-limit violation";
            info.candidates(k).legData = legData;
            continue;
        end

        residuals = localLoopResiduals(commandedPoint, thetaCandidate, params);
        maxResidual = max(abs(residuals));
        if maxResidual > params.residualTol
            info.candidates(k).branchSigns = branchSigns;
            info.candidates(k).theta = thetaCandidate;
            info.candidates(k).valid = false;
            info.candidates(k).reason = "Rejected: residual tolerance exceeded";
            info.candidates(k).maxResidual = maxResidual;
            info.candidates(k).legData = legData;
            continue;
        end

        duplicateIdx = findDuplicateBranch(thetaCandidate, solutions, params.duplicateTol);
        if ~isnan(duplicateIdx)
            info.candidates(k).branchSigns = branchSigns;
            info.candidates(k).theta = thetaCandidate;
            info.candidates(k).valid = false;
            info.candidates(k).reason = "Rejected: duplicate of an already accepted branch";
            info.candidates(k).maxResidual = maxResidual;
            info.candidates(k).legData = legData;
            info.uniqueBranchMap(k) = duplicateIdx;
            continue;
        end

        validCount = validCount + 1;
        solutions(:, validCount) = thetaCandidate; %#ok<AGROW>

        info.candidates(k).branchSigns = branchSigns;
        info.candidates(k).theta = thetaCandidate;
        info.candidates(k).valid = true;
        info.candidates(k).reason = "Accepted";
        info.candidates(k).maxResidual = maxResidual;
        info.candidates(k).legData = legData;
        info.uniqueBranchMap(k) = validCount;

    catch ME
        info.candidates(k).branchSigns = branchSigns;
        info.candidates(k).valid = false;
        info.candidates(k).reason = "Rejected: " + string(ME.message);
    end
end

info.numValidBranches = size(solutions, 2);
info.numRejectedBranches = 8 - info.numValidBranches;
info.validBranches = solutions;

if isempty(solutions)
    error('deltaIKAllBranches:NoValidBranch', ...
        'No valid IK branch found for the requested point.');
end

end

%% ======================= Helpers ==========================
function [theta, legData] = localComputeCandidate(x, y, z, params, branchSigns)
Rb = params.Rb;
Rp = params.Rp;
L  = params.L;
l  = params.l;

theta = zeros(3,1);
legData = repmat(struct('a',nan,'b',nan,'c',nan,'R',nan,'phi',nan,'acosArg',nan),3,1);

[a1,b1,c1] = localLeg1Coeffs(x, y, z, Rb, Rp, L, l);
theta(1) = localSolveLeg(a1, b1, c1, branchSigns(1));
legData(1) = localLegPack(a1,b1,c1);

[a2,b2,c2] = localLeg2Coeffs(x, y, z, Rb, Rp, L, l);
theta(2) = localSolveLeg(a2, b2, c2, branchSigns(2));
legData(2) = localLegPack(a2,b2,c2);

[a3,b3,c3] = localLeg3Coeffs(x, y, z, Rb, Rp, L, l);
theta(3) = localSolveLeg(a3, b3, c3, branchSigns(3));
legData(3) = localLegPack(a3,b3,c3);
end

function pack = localLegPack(a,b,c)
R = hypot(a,b);
phi = atan2(b,a);
pack = struct('a',a,'b',b,'c',c,'R',R,'phi',phi,'acosArg',-c/R);
end

function theta_i = localSolveLeg(a, b, c, branchSign)
R = hypot(a, b);
if R <= eps
    error('Degenerate leg equation with R = 0.');
end
if abs(c) > R + 1e-9
    error('|c| > R, point is unreachable for this leg.');
end
phi = atan2(b, a);
acosArg = max(-1, min(1, -c / R));
if branchSign >= 0
    theta_i = phi + acos(acosArg);
else
    theta_i = phi - acos(acosArg);
end
theta_i = wrapToPiLocal(theta_i);
end

function idx = findDuplicateBranch(thetaCandidate, solutions, duplicateTol)
idx = nan;
if isempty(solutions)
    return;
end
for ii = 1:size(solutions,2)
    if norm(wrapToPiLocal(thetaCandidate - solutions(:,ii))) <= duplicateTol
        idx = ii;
        return;
    end
end
end

function [a1,b1,c1] = localLeg1Coeffs(x, y, z, Rb, Rp, L, l)
a1 = 2*L*(x + Rp - Rb);
b1 = -2*L*z;
c1 = (x + Rp - Rb)^2 + y^2 + z^2 + L^2 - l^2;
end

function [a2,b2,c2] = localLeg2Coeffs(x, y, z, Rb, Rp, L, l)
d2 = x - Rp/2 + Rb/2;
e2 = y + (sqrt(3)/2)*(Rp - Rb);
a2 = L*(-d2 + sqrt(3)*e2);
b2 = -2*L*z;
c2 = d2^2 + e2^2 + z^2 + L^2 - l^2;
end

function [a3,b3,c3] = localLeg3Coeffs(x, y, z, Rb, Rp, L, l)
d3 = x - Rp/2 + Rb/2;
e3 = y - (sqrt(3)/2)*(Rp - Rb);
a3 = L*(-d3 - sqrt(3)*e3);
b3 = -2*L*z;
c3 = d3^2 + e3^2 + z^2 + L^2 - l^2;
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

function tf = isInsideWorkspace(P, params)
x = P(1); y = P(2); z = P(3);
tol = params.tol;

tf = (x >= params.workspace.x(1) - tol) && (x <= params.workspace.x(2) + tol) && ...
     (y >= params.workspace.y(1) - tol) && (y <= params.workspace.y(2) + tol) && ...
     (z >= params.workspace.z(1) - tol) && (z <= params.workspace.z(2) + tol);
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
