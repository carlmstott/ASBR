%Brief: takes robot object and vector of joint angles jointAngles and
%returns stack of matrix exponental forms of twist for each joint WRT body
%frame.
%Params: robot, robot object.
%JointAngles, list of joint angles
%
%Returns: MexpBlist, stack of all of the matrix exponental forms of the twist of
%each joint

function output=MexpBList(robot,jointAngles)


ScrewVectors=[robot.B.w; robot.B.v]; 
MexpBlist=zeros(2,2,length(robot.B)); %faster

%this loop iterates through the screw vectors of each joint and their joint
%angles to create the list of matrix exponental forms of twist, which
%is usefull for creating jacobians
for i=length(robot.B)
    MexpBlist(:,:,i)=transMatExpScrew(ScrewVectors(i),jointAngles(i));
end

output=MexpBlist;
end
