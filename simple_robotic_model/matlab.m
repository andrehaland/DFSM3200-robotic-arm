robot = importrobot('simple_model_generated.urdf')
robot.DataFormat = 'row'
gripper = 'fifth_link'

cupHeight = 0.2;
cupRadius = 0.05;
cupPosition = [-0.05, 0.5, cupHeight / 2];

body = robotics.RigidBody('cupFrame');
setFixedTransform(body.Joint, trvec2tform(cupPosition));
addBody(robot, body, robot.BaseName);

numWaypoints = 4;
q0 = homeConfiguration(robot);
qWaypoints = repmat(q0, numWaypoints, 1);

gik = robotics.GeneralizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'cartesian', 'position', 'aiming', ...
    'orientation', 'joint'});

heightAboveTable = robotics.CartesianBounds(gripper);
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0.05, inf]
                       
distanceFromCup = robotics.PositionTarget('cupFrame');
distanceFromCup.ReferenceBody = gripper;
distanceFromCup.PositionTolerance = 0.05

alignWithCup = robotics.AimingConstraint('fifth_link');
alignWithCup.TargetPoint = [0, 0, 100]

limitJointChange = robotics.JointPositionBounds(robot)

fixOrientation = robotics.OrientationTarget(gripper);
fixOrientation.OrientationTolerance = deg2rad(1)

intermediateDistance = 0.0;

limitJointChange.Weights = zeros(size(limitJointChange.Weights));
fixOrientation.Weights = 0;

distanceFromCup.TargetPosition = [0, 0, intermediateDistance];

[qWaypoints(2, :), solutionInfo] = gik(q0, heightAboveTable, ...
    distanceFromCup, alignWithCup, fixOrientation, ...
    limitJointChange);

limitJointChange.Weights = ones(size(limitJointChange.Weights));
fixOrientation.Weights = 1;

alignWithCup.Weights = 0;

fixOrientation.TargetOrientation = ...
    tform2quat(getTransform(robot, qWaypoints(2, :), gripper));

finalDistanceFromCup = 0.005;
distanceFromCupValues = linspace(intermediateDistance, finalDistanceFromCup, numWaypoints - 1);

maxJointChange = deg2rad(20);

for k = 3 : numWaypoints
    distanceFromCup.TargetPosition(3) = distanceFromCupValues(k - 1);
    limitJointChange.Bounds = [qWaypoints(k - 1, :)' - maxJointChange, ...
        qWaypoints(k - 1, :)' + maxJointChange];
    [qWaypoints(k, :), solutionInfo] = gik(qWaypoints(k - 1, :), ...
        heightAboveTable, distanceFromCup, alignWithCup, ...
        fixOrientation, limitJointChange);
end

framerate = 15;
r = robotics.Rate(framerate);
tFinal = 10;
tWaypoints = [0, linspace(tFinal / 2, tFinal, size(qWaypoints, 1) - 1)];
numFrames = tFinal * framerate;
qInterp = pchip(tWaypoints, qWaypoints', linspace(0, tFinal, numFrames))';

gripperPosition = zeros(numFrames, 3);

for k = 1 : numFrames
    gripperPosition(k, :) = tform2trvec(getTransform(robot, qInterp(k, :), ...
        gripper));
end

figure;
show(robot, qWaypoints(1, :), 'PreservePlot', false);
hold on;
exampleHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition);
p = plot3(gripperPosition(1, 1), gripperPosition(1, 2), gripperPosition(1, 3));

for k = 1 : size(qInterp, 1)
    show(robot, qInterp(k, :), 'PreservePlot', false);
    p.XData(k) = gripperPosition(k, 1);
    p.YData(k) = gripperPosition(k, 2);
    p.ZData(k) = gripperPosition(k, 3);
    waitfor(r);
end
hold off