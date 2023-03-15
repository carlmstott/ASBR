function BodyK=FK_body(robot)

M=robot.M; %we're gonna need this

robot=robot; %im not sure if I need to do this or not, Bhrat will probably
               %know
%this function takes in the robot object and then spits out the body form
%kinumatics of the end effector. 

%first thing we need is to get the 4x4 matrix exponental forms of the body
%twist vectors of our robot. Thankfully we already wrote a fucntion that
%does this for us ;)
MexpBs=MexpB(robot); 

%according to slide14 w6L2, we have to multiply all of these matrix
%exponental forms together, done below
PostMult=eye(4);
for i=length(MexpBs)
PostMult=PostMult*MexpBs(:,:,i);
end

%so this function gives us a stack of matricies i deep, where i is the
%amount of joints. i=1 is joint 1, i=2 is joint 2, etc etc


BodyK=M*PostMult;
end
