%CurrentConfig is currentconfiguration of robot, IE a list of joint
%angles. It starts as being the initial configuration, and then changes as
%we iterate through the loop.
%xd is the desired end effector configuration
%endConfig is the vector or joint angles that position the robot such that
%the end effector is located and oriented to xd.

function EndConfig=J_inverse_kinumatics(robot, CurrentConfig, xd)
i=1; %counter, used to make sure I only iterate a maximum of 2 times

[TSB,~]=FK_space(robot,CurrentConfig,1); %w8L1S14, looking at the diagram
                                %it is clear that TSB is 


Tbd=(double(TSB)^-1) * xd; %Tbd=Tbs*Tsd

Vb=MatLog(Tbd); %W8L1S14, need twist vector of tbd for math in below line
%to work out



while  i < 400 || rad2deg(norm(Vb(1:3)))>5 && norm(Vb(4:6)) > norm(robot.M([1:3],end))*.02  %2 error in position, 5 degree error in
                               %orientation
norm(Vb)
%we need to calculate Vb in order to update our current configuration.
%I am using W8L1S14.

J=J_body(robot, CurrentConfig);
Jdagger=double(pinv(J)); %matlab pinv function makes the psudoinverse no
%matter what the shape 


%these lines follow the IK algorithem described in w8L1S14

CurrentConfig=CurrentConfig+Jdagger*Vb; %updating the current config

[TSB,~]=FK_space(robot,CurrentConfig,1); %recalculating TSB with new  
                                            %configuration

Tbd=((double(TSB))^-1)*xd; %Tbd=(Tsb^-1)*xd (xd is Tsd), 


Vb=MatLog(Tbd); %W8L2S14, recalculating

normStoreRot(i)=rad2deg(norm(Vb(1:3)));
normStoreTranslation(i)=norm(Vb(4:6));
i=i+1
end

EndConfig=CurrentConfig;


end