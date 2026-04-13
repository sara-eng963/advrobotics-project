function [thetaBest, info] = deltaIK(x, y, z, params, thetaRef)
% deltaIK  Branch-aware inverse kinematics for the delta robot.
%
%   [thetaBest, info] = deltaIK(x, y, z, params)
%   [thetaBest, info] = deltaIK(x, y, z, params, thetaRef)
%
%   INPUTS
%       x, y, z  : Desired platform-center coordinates [m]
%       params   : Structure returned by deltaParams()
%       thetaRef : Optional 3x1 reference posture [rad]. If provided, only
%                  same-branch-consistent solutions are accepted.
%
%   OUTPUTS
%       thetaBest : Selected 3x1 IK solution [rad]
%       info      : Diagnostic structure with all valid branches, branch
%                   selection costs, and selected branch metadata.
%
%   BRANCH SELECTION STRATEGY
%       1) Enumerate all unique valid IK branches.
%       2) Compute wrapped-angle distance to a reference posture.
%       3) If thetaRef is supplied, reject all branches outside the
%          same-branch tolerance.
%       4) Otherwise pick the closest valid branch to the configured
%          default branch preference.

arguments
    x (1,1) double
    y (1,1) double
    z (1,1) double
    params (1,1) struct
    thetaRef (3,1) double = NaN(3,1)
end

[validBranches, allInfo] = deltaIKAllBranches(x, y, z, params);
numBranches = size(validBranches, 2);

info = struct();
info.commandedPoint = [x; y; z];
info.validBranches = validBranches;
info.allBranchInfo = allInfo;
info.numValidBranches = numBranches;
info.selectionCosts = nan(1, numBranches);
info.selectedBranchIndex = nan;
info.selectedReference = "";
info.sameBranchRequired = false;
info.consistentBranchMask = false(1, numBranches);
info.selectedMaxResidual = nan;

if all(isnan(thetaRef))
    thetaRef = params.defaultBranchPreference;
    info.selectedReference = "defaultBranchPreference";
else
    info.selectedReference = "userSuppliedThetaRef";
    info.sameBranchRequired = true;
end

for k = 1:numBranches
    err = wrapToPiLocal(validBranches(:,k) - thetaRef);
    info.selectionCosts(k) = norm(err);
end

if info.sameBranchRequired
    info.consistentBranchMask = info.selectionCosts <= params.branchMatchTol;
    consistentIdx = find(info.consistentBranchMask);

    if isempty(consistentIdx)
        error('deltaIK:NoMatchingBranch', ...
            ['No valid IK branch matches the supplied reference posture ' ...
             'within branchMatchTol = %.3e rad.'], params.branchMatchTol);
    end

    [~, localMinIdx] = min(info.selectionCosts(consistentIdx));
    idx = consistentIdx(localMinIdx);
else
    info.consistentBranchMask(:) = true;
    [~, idx] = min(info.selectionCosts);
end

info.selectedBranchIndex = idx;
thetaBest = validBranches(:, idx);
info.selectedTheta = thetaBest;

selectedResiduals = localLoopResiduals([x; y; z], thetaBest, params);
info.selectedLoopResiduals = selectedResiduals;
info.selectedMaxResidual = max(abs(selectedResiduals));

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

res = [norm(C1 - B1)^2 - params.l^2;
       norm(C2 - B2)^2 - params.l^2;
       norm(C3 - B3)^2 - params.l^2];
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
