% brief: calculates forward kinematics in space frame
% params:
% robot: robot object consistig of the robot's kinematic properties
% defined in defineRobot.m 
% jointAngles: list of joint angles 0 to n
% returns:
% T: forward kinematics transformation matrix in space frame 4x4
% jointToJointTransforms: 4x4xn dimensional matrix where each 4x4 matrix
% represents transform from frame n-1 to frame n.
% err: error code
%if you are feeding symbolics in here, change the symbolic varable to be 1


function [T, jointToJointTransforms, err] = FK_space(robot,jointAngles, symbolic)


if length(jointAngles) ~= robot.numJoints
    error("Error: make sure vector of angles has same length as robot has joints")
    err = -1; %#ok<UNRCH>
    return
end

% calculate screw matrix exponentials (4x4) and multiply them from joint 1
% to n. e_S1_theta1 * e_S2_theta2 ... * e_Sn_thetan . Ref: W6-L2 slide 5

MatrixExponentals=eye(4);

for i = 1 : robot.numJoints
    twist = robot.S(:, i);
    e_S_theta = transMatExpScrew(twist, jointAngles(i));
    MatrixExponentals = MatrixExponentals * e_S_theta;
    jointToJointTransforms(:,:,i) = e_S_theta;


     if symbolic == 0
     jointToJointTransformsSE3(i) = se3(double(e_S_theta)); %#ok<AGROW,NASGU> the se3 function turns our 4x4 matrix into an se3 object, which we use to plot
     end

     

end

% product of exponentials (e_S1_theta1 * e_S2_theta2 ... * e_Sn_thetan) * M . Ref: W6-L2 slide 5
T = MatrixExponentals * robot.M;
err = 0;



if symbolic == 0
Tvectors=trvec(jointToJointTransforms); %extracts the translation vectors from the se3 objects, used for plotting
name=["frame1";"frame2";"frame3"];
plotTransforms(jointTransformations,'FrameAxisLabels',"off", 'FrameLabel',name)
hold
plot3(Tvectors(:,1),Tvectors(:,2),Tvectors(:,3))
end

end