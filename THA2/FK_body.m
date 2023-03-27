%This function was written by carl stott on 3/15
%Brief: takes robot object and vector of joint angles jointAngles and
%returns the fwd kinumatics in the end effector frame
%Params: robot, robot object.
%JointAngles, list of joint angles
%
%Returns: BodyK, the orientation of the end effector
%MexpBlist, stack of all of the matrix exponental forms of the twist of
%each joint
function BodyK=FK_body(robot,jointAngles)

if length(jointAngles) ~=length(robot.B(1,:))
    error("make sure vector of angles has same length as robot has joints")
end

M=robot.M;
ScrewVectors=robot.B;
MatrixExponentals=eye(4);


%this loop iterates through the screw vectors of each joint and their joint
%angles to create the correcsponding matrix exponental representation, and
%then multiplys them all together starting with joint 1, reference: slide14
% w6L2

for i=1:robot.numJoints
    MatrixExponentals=MatrixExponentals*transMatExpScrew(ScrewVectors(:,i),jointAngles(i));
end


BodyK=M*MatrixExponentals;
end

