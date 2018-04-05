sub = rossubscriber('MATLAB/cartesian_subscriber', 'geometry_msgs/Vector3', @k_func);
pub = rospublisher('MATLAB/kinematic_publisher', 'std_msgs/Float64MultiArray');

function k_func(~, msg)
    % Setup publisher which publishes the calculated joint configurations
    pub = rospublisher('MATLAB/kinematic_publisher', 'std_msgs/Float64MultiArray');
    
    % Import URDF to create a rigid body tree for solving kinematics
    % Needs to be changed to absolute path to URDF
    robot = importrobot('/home/andre/catkin_ws/src/DFSM3200-robotic-arm/simple_robotic_model/urdf/simple_model_generated.urdf');
    
    % Define end-effector
    ee = 'end_effector';

    % Use data from Vector3 from subscriber in transformation matrix
    tform = [0 -1 0 msg.X
             1 0 0 msg.Y
             0 0 1 msg.Z
             0 0 0 1];

    % Solve inverse kinematics
    ik = robotics.InverseKinematics('RigidBodyTree', robot);
    
    ik.SolverParameters.MaxIterations = 1500; % Default 1500
    
    weights = [0.25 0.25 0.25 1 1 1];
    initialguess = robot.homeConfiguration;
    [configSoln, solnInfo] = ik(ee, tform, weights, initialguess);
    
    response = rosmessage('std_msgs/Float64MultiArray');
    response.Data = [
                    configSoln(1).JointPosition
                    configSoln(2).JointPosition
                    configSoln(3).JointPosition
                    configSoln(4).JointPosition
                    configSoln(5).JointPosition
                    ];
    send(pub, response);
end
