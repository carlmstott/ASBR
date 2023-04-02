% brief: calculates forward kinematics in space frame
% params:
% robot: robot object consistig of the robot's kinematic properties
% defined in defineRobot.m
% jointAngles: list of joint angles 0 to n
% returns:
% T: forward kinematics transformation matrix in space frame 4x4
% jointToJointTransforms: 4x4xn dimensional matrix where each 4x4 matrix
% represents transform from frame n-1 to frame n
% spaceToJointTransforms: 4x4xn dimensional matrix where each 4x4 matrix
% represents transform from the spatial reference frame to frame n
% err: error code

function [T, jointToJointTransforms, spaceToJointTransforms, err] = FK_space(robot,jointAngles, plot)

if length(jointAngles) ~= robot.numJoints
    warning("Error: make sure vector of angles has same length as robot has joints")
    err = -1;
    return
end

% calculate screw matrix exponentials (4x4) and multiply them from joint 1
% to n. e_S1_theta1 * e_S2_theta2 ... * e_Sn_thetan . Ref: W6-L2 slide 5

MatrixExponentals=eye(4);

for i = 1 : robot.numJoints
    twist = robot.S(:, i);
    e_S_theta = transMatExpScrew(twist, jointAngles(i));
    MatrixExponentals = MatrixExponentals * e_S_theta;
    jointToJointTransforms(:, :, i) = e_S_theta;
    spaceToJointTransforms(:, :, i) = MatrixExponentals * robot.M;
end

% product of exponentials (e_S1_theta1 * e_S2_theta2 ... * e_Sn_thetan) * M . Ref: W6-L2 slide 5
T = MatrixExponentals * robot.M;

% convert all outputs to double if using numeric values
if(isnumeric(T))
    T = double(T);
    jointToJointTransforms = double(jointToJointTransforms);
    spaceToJointTransforms = double(spaceToJointTransforms);
end

% if plotting is enabled
if(plot)
    figure; hold on; grid on;
    if(isnumeric(spaceToJointTransforms))
        name = "";
        for i = 1 : robot.numJoints
            jointToJointTransforms_SE3(i) = se3(jointToJointTransforms(:, :, i));
            spaceToJointTransforms_SE3(i) = se3(spaceToJointTransforms(:, :, i));
            name(i)=strcat('joint_', num2str(i));
        end
        plotTransforms(spaceToJointTransforms_SE3,'FrameAxisLabels',"off",'FrameLabel',name)
        Tvectors=trvec(spaceToJointTransforms_SE3);
        plot3(Tvectors(:,1),Tvectors(:,2),Tvectors(:,3))

    else
        warning("Error: cannot plot symbolic values. Input numeric values to forward kinematics")
        err = -2;
        return
    end
end

err = 0;
end