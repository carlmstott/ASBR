% brief: takes robot object and vector of joint angles jointAngles and
% returns the fwd kinumatics in the end effector frame
% params:
% robot, robot object.
% jointAngles: list of joint angles 0 to n
% plot: boolean value to plot (or not) the robot's frames
%
%Returns:
% BodyK, the orientation of the end effector
% jointToJointTransforms: 4x4xn dimensional matrix where each 4x4 matrix
% represents transform from frame n to frame n-1
% eeToJointTransforms: 4x4xn dimensional matrix where each 4x4 matrix
% represents transform from the end-effector reference frame to frame n
% err: error code

function [BodyK, err] = FK_body(robot,jointAngles, plot)

if length(jointAngles) ~= robot.numJoints
    warning("Error: make sure vector of angles has same length as robot has joints")
    err = -1;
    return
end

M=robot.M;
ScrewVectors=robot.B;
MatrixExponentals=eye(4);


%this loop iterates through the screw vectors of each joint and their joint
%angles to create the correcsponding matrix exponental representation, and
%then multiplys them all together starting with joint 1, reference: slide14
%w6L2

for i = 1 : robot.numJoints
    EStheta = transMatExpScrew(ScrewVectors(:,i), jointAngles(i));
    MatrixExponentals = MatrixExponentals * EStheta;
    jointToJointTransforms(:, :, i) = EStheta; %#ok<*AGROW>
end

BodyK = M(:,:,end) * MatrixExponentals; %this accounts for a robot object
%that has an M for each joint, or a single M for the end effector.
jointToJointTransforms(:, :, end)= BodyK;

% convert all outputs to double if using numeric values



% if plotting is enabled
if(plot)
    figure; hold on; grid on;
    if(isnumeric(jointToJointTransforms))
        for i = 1 : robot.numJoints
            jointToJointTransforms_SE3(i) = se3(jointToJointTransforms(:, :, i));
            name(i)=strcat("joint_", num2str(i));
        end
        plotTransforms(jointToJointTransforms_SE3,'FrameAxisLabels',"off",'FrameLabel',name)
        Tvectors=trvec(jointToJointTransforms_SE3);
        plot3(Tvectors(:,1),Tvectors(:,2),Tvectors(:,3))

    else
        warning("Error: cannot plot symbolic values. Input numeric values to forward kinematics")
        err = -2;
        return
    end
end

err = 0;
end