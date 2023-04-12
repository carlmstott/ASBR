%this is the exact same function as FK_space, but plots the locations of
%all of the joints instead of only the end effector and the base joint


function [EndEffectorT, spaceToJointTransformsSE3, err] = Fk_Space_for_Kuka(robot,jointAngles, symbolic)


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

    if symbolic ==1
    jointToJointTransforms(:,:,i) = e_S_theta;

    else
     jointToJointTransformsSE3(i) = se3(double(e_S_theta)); %#ok<AGROW,NASGU> the se3 function turns our 4x4 matrix into an se3 object, which we use to plot
     spaceToJointTransformsSE3(i)=se3(double(MatrixExponentals)*robot.M(:,:,i)); % product of exponentials (e_S1_theta1 * e_S2_theta2 ... * e_Sn_thetan) * M . Ref: W6-L2 slide 5
     end

     

end
err = 0;



if symbolic == 0
%hold %this clears the FKspace plot from the past
Tvectors=trvec(spaceToJointTransformsSE3); %extracts the translation vectors from the se3 objects, used for plotting
name=["F1";"F2";"F3";"F4";"F5";"F6"];
plotTransforms(spaceToJointTransformsSE3,'FrameAxisLabels',"off",'FrameLabel',name)
xlim([-20 20]);
ylim([-20 20]);
zlim([-20 20]);
hold ON
plot3(Tvectors(:,1),Tvectors(:,2),Tvectors(:,3))
end
EndEffectorT=tform(spaceToJointTransformsSE3(6))
end