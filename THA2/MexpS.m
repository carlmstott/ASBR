function output=MexpS(robot)

%this fucntion calcualtes the 4x4 matrix exponental form of the spacial
%twist vectors of my robot

theta = sym('theta', [robot.numJoints 1]); %for now our theta's are
    %symbolic, this will change later


for i=theta.length 

%theta is the angle being applied to the twist vector

w=robot.S(i).w;
v=robot.S(i).s;


%ok so there are 2 conditions, condition #1 is if |w|=1
if isequal(w,[0;0;0]) 

UpperRTerm=(eye(3)*theta(i)+(1-cos(theta(i))*w+(theta-sin(theta(i))))*(w*w))*v;
%this is upper right term of matrix exponental eq

%now to do rodriguez on skew(w)
Sw=skew(w);
TopRight3x3=eye(3)+sin(theta(i))*Sw+(1-cos(theta(i))*(Sw*Sw));


else %if we get in here, the joint is prysmatic and the way to get
     %exponenta form of the twist vector is different.

UpperRTerm=v*theta;

TopRight3x3=eye(3);
end
    
output(:,:,i)=[TopRight3x3 UpperRTerm;0 0 0 1];
end

end
