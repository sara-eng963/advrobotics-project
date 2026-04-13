%% test_delta.m
% Delta Robot Milestone 1 Validation Script
%
% This script is focused on Milestone 1, Point 4:
%   Simulation-Based Validation
%       I.   p = (x, y, z)
%       II.  IK: (theta1, theta2, theta3) = IK(x, y, z)
%       III. FK: p_fk = f(theta1, theta2, theta3)
%       IV.  Compare with the original target
%
% It also supports the complementary consistency test:
%   theta -> FK -> IK
%
% Main outputs:
%   - pointwise validation for one Cartesian target
%   - pointwise validation for one joint target
%   - batch statistics over multiple random Cartesian targets
%   - batch statistics over multiple random joint targets
%   - saved summary structure for report generation

clear; clc; close all;

params = deltaParams();
rng(params.validation.randomSeed);

fprintf('============================================================\n');
fprintf(' Delta Robot - Milestone 1 Validation\n');
fprintf('============================================================\n');

%% ----------------------------------------------------------
% CASE A: p -> IK -> FK
% -----------------------------------------------------------
fprintf('\nCASE A: Cartesian target -> IK -> FK -> compare with target\n');
fprintf('------------------------------------------------------------\n');

p_cmd = params.validation.defaultPoint;
[theta_from_IK, ikInfo_A] = deltaIK(p_cmd(1), p_cmd(2), p_cmd(3), params);
[p_from_FK_A, fkInfo_A]   = deltaFK(theta_from_IK(1), theta_from_IK(2), theta_from_IK(3), params);

e_cons_A = p_from_FK_A - p_cmd;
e_cons_A_norm = norm(e_cons_A);

fprintf('Requested point p_cmd [m]           = [%.6f, %.6f, %.6f]\n', p_cmd(1), p_cmd(2), p_cmd(3));
fprintf('IK solution theta [deg]             = [%.6f, %.6f, %.6f]\n', ...
    rad2deg(theta_from_IK(1)), rad2deg(theta_from_IK(2)), rad2deg(theta_from_IK(3)));
fprintf('Recovered FK point p_fk [m]         = [%.6f, %.6f, %.6f]\n', ...
    p_from_FK_A(1), p_from_FK_A(2), p_from_FK_A(3));
fprintf('Consistency error e_cons [m]        = [%.6e, %.6e, %.6e]\n', ...
    e_cons_A(1), e_cons_A(2), e_cons_A(3));
fprintf('Consistency error norm ||e_cons||   = %.6e m\n', e_cons_A_norm);
fprintf('Valid IK branches                   = %d\n', ikInfo_A.numValidBranches);
fprintf('Selected IK branch index            = %d\n', ikInfo_A.selectedBranchIndex);
fprintf('FK max loop residual                = %.3e\n', fkInfo_A.maxResidual);
fprintf('IK max loop residual                = %.3e\n', ikInfo_A.selectedMaxResidual);
fprintf('FK determinant det(A)               = %.3e\n', fkInfo_A.detA);
fprintf('FK discriminant                     = %.3e\n', fkInfo_A.discriminant);

%% ----------------------------------------------------------
% CASE B: theta -> FK -> IK
% -----------------------------------------------------------
fprintf('\nCASE B: Joint target -> FK -> IK -> compare with original branch\n');
fprintf('------------------------------------------------------------\n');

theta_cmd = deg2rad(params.validation.defaultThetaDeg);
[p_from_FK_B, fkInfo_B] = deltaFK(theta_cmd(1), theta_cmd(2), theta_cmd(3), params);
[theta_from_IK_B, ikInfo_B] = deltaIK(p_from_FK_B(1), p_from_FK_B(2), p_from_FK_B(3), params, theta_cmd);

e_theta_B = wrapToPiLocal(theta_from_IK_B - theta_cmd);
e_theta_B_norm = norm(e_theta_B);

fprintf('Requested theta_cmd [deg]           = [%.6f, %.6f, %.6f]\n', ...
    rad2deg(theta_cmd(1)), rad2deg(theta_cmd(2)), rad2deg(theta_cmd(3)));
fprintf('FK position from theta_cmd [m]      = [%.6f, %.6f, %.6f]\n', ...
    p_from_FK_B(1), p_from_FK_B(2), p_from_FK_B(3));
fprintf('Recovered IK theta [deg]            = [%.6f, %.6f, %.6f]\n', ...
    rad2deg(theta_from_IK_B(1)), rad2deg(theta_from_IK_B(2)), rad2deg(theta_from_IK_B(3)));
fprintf('Joint consistency error [deg]       = [%.6e, %.6e, %.6e]\n', ...
    rad2deg(e_theta_B(1)), rad2deg(e_theta_B(2)), rad2deg(e_theta_B(3)));
fprintf('Joint consistency norm              = %.6e rad\n', e_theta_B_norm);
fprintf('Valid IK branches                   = %d\n', ikInfo_B.numValidBranches);
fprintf('Selected matching branch index      = %d\n', ikInfo_B.selectedBranchIndex);
fprintf('FK max loop residual                = %.3e\n', fkInfo_B.maxResidual);
fprintf('IK max loop residual                = %.3e\n', ikInfo_B.selectedMaxResidual);

%% ----------------------------------------------------------
% BATCH C: random Cartesian targets -> IK -> FK
% -----------------------------------------------------------
fprintf('\nBATCH C: Random Cartesian targets -> IK -> FK\n');
fprintf('------------------------------------------------------------\n');

nCart = params.validation.numCartesianTests;
cartResults = initCartesianResults(nCart);

for k = 1:nCart
    pTry = sampleWorkspacePoint(params);
    cartResults.p_cmd(:,k) = pTry;

    try
        [thetaTry, ikInfoTry] = deltaIK(pTry(1), pTry(2), pTry(3), params);
        [pRec, fkInfoTry] = deltaFK(thetaTry(1), thetaTry(2), thetaTry(3), params);

        cartResults.theta(:,k)         = thetaTry;
        cartResults.p_fk(:,k)          = pRec;
        cartResults.errVec(:,k)        = pRec - pTry;
        cartResults.errNorm(k)         = norm(cartResults.errVec(:,k));
        cartResults.ikResidual(k)      = ikInfoTry.selectedMaxResidual;
        cartResults.fkResidual(k)      = fkInfoTry.maxResidual;
        cartResults.selectedBranch(k)  = ikInfoTry.selectedBranchIndex;
        cartResults.numValidBranches(k)= ikInfoTry.numValidBranches;
        cartResults.success(k)         = true;
        cartResults.message(k)         = "OK";
    catch ME
        cartResults.message(k) = string(ME.message);
    end
end

reportBatchStats('Batch C (p -> IK -> FK)', cartResults.errNorm, cartResults.success, 'm');

%% ----------------------------------------------------------
% BATCH D: random joint targets -> FK -> IK
% -----------------------------------------------------------
fprintf('\nBATCH D: Random joint targets -> FK -> IK\n');
fprintf('------------------------------------------------------------\n');

nJoint = params.validation.numJointTests;
jointResults = initJointResults(nJoint);

for k = 1:nJoint
    thetaTry = sampleJointAngles(params);
    jointResults.theta_cmd(:,k) = thetaTry;

    try
        [pTry, fkInfoTry] = deltaFK(thetaTry(1), thetaTry(2), thetaTry(3), params);
        [thetaRec, ikInfoTry] = deltaIK(pTry(1), pTry(2), pTry(3), params, thetaTry);

        jointResults.p_fk(:,k)           = pTry;
        jointResults.theta_rec(:,k)      = thetaRec;
        jointResults.errVec(:,k)         = wrapToPiLocal(thetaRec - thetaTry);
        jointResults.errNorm(k)          = norm(jointResults.errVec(:,k));
        jointResults.ikResidual(k)       = ikInfoTry.selectedMaxResidual;
        jointResults.fkResidual(k)       = fkInfoTry.maxResidual;
        jointResults.selectedBranch(k)   = ikInfoTry.selectedBranchIndex;
        jointResults.numValidBranches(k) = ikInfoTry.numValidBranches;
        jointResults.success(k)          = true;
        jointResults.message(k)          = "OK";
    catch ME
        jointResults.message(k) = string(ME.message);
    end
end

reportBatchStats('Batch D (theta -> FK -> IK)', jointResults.errNorm, jointResults.success, 'rad');

%% ----------------------------------------------------------
% Plots
% -----------------------------------------------------------
figure('Name', 'Delta Robot Milestone 1 Validation', 'Color', 'w');

subplot(2,1,1);
validC = cartResults.success;
stem(find(validC), cartResults.errNorm(validC), 'filled', 'LineWidth', 1.2);
grid on;
xlabel('Valid Cartesian test index');
ylabel('||e_{cons}|| [m]');
title('Batch C: Cartesian consistency error');

subplot(2,1,2);
validD = jointResults.success;
stem(find(validD), jointResults.errNorm(validD), 'filled', 'LineWidth', 1.2);
grid on;
xlabel('Valid joint test index');
ylabel('Joint error norm [rad]');
title('Batch D: Same-branch recovery error');

%% ----------------------------------------------------------
% Save summary for report integration
% -----------------------------------------------------------
summary = struct();
summary.params = params;
summary.caseA.p_cmd = p_cmd;
summary.caseA.theta_from_IK = theta_from_IK;
summary.caseA.p_from_FK = p_from_FK_A;
summary.caseA.errVec = e_cons_A;
summary.caseA.errNorm = e_cons_A_norm;
summary.caseA.ikInfo = ikInfo_A;
summary.caseA.fkInfo = fkInfo_A;

summary.caseB.theta_cmd = theta_cmd;
summary.caseB.p_from_FK = p_from_FK_B;
summary.caseB.theta_from_IK = theta_from_IK_B;
summary.caseB.errVec = e_theta_B;
summary.caseB.errNorm = e_theta_B_norm;
summary.caseB.ikInfo = ikInfo_B;
summary.caseB.fkInfo = fkInfo_B;

summary.batchC = buildBatchSummary(cartResults.errNorm, cartResults.success);
summary.batchD = buildBatchSummary(jointResults.errNorm, jointResults.success);
summary.cartResults = cartResults;
summary.jointResults = jointResults;

save('delta_validation_summary.mat', 'summary');

%% ----------------------------------------------------------
% Overall summary
% -----------------------------------------------------------
fprintf('\n============================================================\n');
fprintf(' Overall Summary\n');
fprintf('============================================================\n');
fprintf('Case A mean requirement direction   : p -> IK -> FK\n');
fprintf('Case A ||e_cons||                   : %.6e m\n', e_cons_A_norm);
fprintf('Case B theta recovery norm          : %.6e rad\n', e_theta_B_norm);
fprintf('Batch C successful cases            : %d / %d\n', nnz(cartResults.success), nCart);
fprintf('Batch D successful cases            : %d / %d\n', nnz(jointResults.success), nJoint);
fprintf('Batch C mean / max error            : %.6e / %.6e m\n', ...
    summary.batchC.meanError, summary.batchC.maxError);
fprintf('Batch D mean / max error            : %.6e / %.6e rad\n', ...
    summary.batchD.meanError, summary.batchD.maxError);
fprintf('Saved summary file                  : delta_validation_summary.mat\n');

%% ===================== Local functions ====================
function s = initCartesianResults(n)
s = struct();
s.p_cmd           = nan(3,n);
s.theta           = nan(3,n);
s.p_fk            = nan(3,n);
s.errVec          = nan(3,n);
s.errNorm         = nan(1,n);
s.ikResidual      = nan(1,n);
s.fkResidual      = nan(1,n);
s.selectedBranch  = nan(1,n);
s.numValidBranches= nan(1,n);
s.success         = false(1,n);
s.message         = strings(1,n);
end

function s = initJointResults(n)
s = struct();
s.theta_cmd       = nan(3,n);
s.p_fk            = nan(3,n);
s.theta_rec       = nan(3,n);
s.errVec          = nan(3,n);
s.errNorm         = nan(1,n);
s.ikResidual      = nan(1,n);
s.fkResidual      = nan(1,n);
s.selectedBranch  = nan(1,n);
s.numValidBranches= nan(1,n);
s.success         = false(1,n);
s.message         = strings(1,n);
end

function p = sampleWorkspacePoint(params)
x = params.workspace.x(1) + diff(params.workspace.x) * rand();
y = params.workspace.y(1) + diff(params.workspace.y) * rand();
z = params.workspace.z(1) + diff(params.workspace.z) * rand();
p = [x; y; z];
end

function theta = sampleJointAngles(params)
mins = params.thetaLimits(:,1);
maxs = params.thetaLimits(:,2);
theta = mins + (maxs - mins) .* rand(3,1);
end

function reportBatchStats(name, errNorm, successMask, unitLabel)
validErr = errNorm(successMask);
failed = nnz(~successMask);

fprintf('%s\n', name);
fprintf('  Successful cases : %d\n', numel(validErr));
fprintf('  Failed cases     : %d\n', failed);

if isempty(validErr)
    fprintf('  No valid cases available for statistics.\n');
    return;
end

fprintf('  Mean error       : %.6e %s\n', mean(validErr), unitLabel);
fprintf('  Max  error       : %.6e %s\n', max(validErr), unitLabel);
fprintf('  Std  error       : %.6e %s\n', std(validErr), unitLabel);
end

function out = buildBatchSummary(errNorm, successMask)
validErr = errNorm(successMask);
out = struct();
out.successCount = nnz(successMask);
out.failCount    = nnz(~successMask);
out.meanError    = mean(validErr, 'omitnan');
out.maxError     = max(validErr, [], 'omitnan');
out.stdError     = std(validErr, 0, 'omitnan');
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
