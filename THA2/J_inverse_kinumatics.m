%CurrentConfig is currentconfiguration of robot, IE a list of joint
%angles. It starts as being the initial configuration, and then changes as
%we iterate through the loop.
%xd is the desired end effector configuration
%endConfig is the vector or joint angles that position the robot such that
%the end effector is located and oriented to xd.

function EndConfig=J_inverse_kinumatics(robot, CurrentConfig, xd)
i=1; %counter, used to make sure I only iterate a maximum of 2 times

[FKBody,~]=FK_body(robot,CurrentConfig,0);
Tbd=((double(FKBody))^-1)*xd; %Tbd=Tbs*Tsd

Vb=MatLog(Tbd); %W8L2S14, need twist vector of tbd for math in below line
%to work out

while  i < 400 || norm(Vb)>1 %will comment back in

%we need to calculate Vb in order to update our current configuration.
%I am using W8L1S14.

J=J_body(robot, CurrentConfig);
Jt=transpose(J);


if robot.numJoints > 6 %meaning we have a fat robot (W8L1S11)
    Jdagger=Jt*((J*Jt)^-1);
end
if robot.numJoints < 6 %meaning we have a tall robot (W8L1S11)
    Jdagger=((Jt*J)^-1)*Jt;
end
if robot.numJoints == 6 %
    Jdagger=J;
end
Jdagger=double(Jdagger);

Jdagger=double(pinv(J));



CurrentConfig=CurrentConfig+Jdagger*Vb; %updating the current config

[FKBody,~]=FK_body(robot,CurrentConfig,0); %recalculating 


%we need to calculate Vb in order to update our current configuration.
%I am using W8L1S14.
Tbd=((double(FKBody))^-1)*xd; %Tbd=Tbs*Tsd
Vb=MatLog(Tbd); %W8L2S14, need twist vector of tbd for math in below line
%to work out


i=i+1
normStore(i)=norm(Vb)
end

EndConfig=CurrentConfig;


end