% brief: calculates forward kinematics in space frame
% params:
% robot: robot object consistig of the robot's kinematic properties
% defined in defineRobot.m
% jointAngles: list of joint angles 0 to n
% plot: boolean value to plot (or not) the robot's frames
% returns:
% T: forward kinematics transformation matrix in space frame 4x4
% jointToJointTransforms: 4x4xn dimensional matrix where each 4x4 matrix
% represents transform from frame n-1 to frame n
% spaceToJointTransforms: 4x4xn dimensional matrix where each 4x4 matrix
% represents transform from the spatial reference frame to frame n
% err: error code

function [SpaceK, err] = FK_space(robot,jointAngles, plot)

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
end

% product of exponentials (e_S1_theta1 * e_S2_theta2 ... * e_Sn_thetan) * M . Ref: W6-L2 slide 5
SpaceK = MatrixExponentals * robot.M(:,:,end);
jointToJointTransforms(:, :, end)= SpaceK;

% convert output to double if using numeric values


% if plotting is enabled
if(plot)
    jointToJointTransforms=double(jointToJointTransforms);
    figure; hold on; grid on;
    if(isnumeric(jointToJointTransforms))
        basejoint_SE3=se3(jointToJointTransforms(:, :, 1));
        endeffector_SE3 = se3(jointToJointTransforms(:, :, end));
            plotTransforms(basejoint_SE3,'FrameAxisLabels',"off",'FrameLabel','frame1')
            plotTransforms(endeffector_SE3,'FrameAxisLabels',"off",'FrameLabel','end_effector')
    else
        warning("Error: cannot plot symbolic values. Input numeric values to forward kinematics")
        err = -2;
        return
    end
end

err = 0;
end