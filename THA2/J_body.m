%Brief: creates the body jacobian for the robot's current orientation
%
%Params: robot, robot object.
%JointAngles, list of joint angles
%
%Returns: jac, body jacobian

function jac=J_body(robot, jointAngles)
r=robot;
 
j=length(r.B); %number of joints
T=eye(4); %placeholder
jac=zeros(6,j); %computaonally faster


for i=j:-1:1 %this loop will increment backward, so I can caululate the
             %last column of my body jacobian first.

if i==j %as the final column of the body jacobain is just the final twist
        %vector, im going to attack it by itseld
jac(:,j)=B(:,j);
else

T=T*inv(MexpBlist(:,:,i+1)); %this is the matrix im going to be taking the adjoint of
                            %I am referencing W7L2 slide 7 here. This
                            %should increment such that I tack on the
                            %inverse of e^-1(B(i+1))(theta+1) on the end of
                            %my last iteration's T

jac(:,i)=Adj(T)*B(i);       %calculating my ith column of my body jacobian

end
end


end




