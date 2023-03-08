function output=Mexp(robot)

theta = sym('theta', [robot.numJoints 1]);

for i=theta.length 

%theta is the angle being applied to the twist vector

w=robot.S(i).w;
v=robot.S(i).s;

UpperRTerm=(eye(3)*theta(i)+(1-cos(theta(i))*w+(theta-sin(theta(i))))*(w*w))*v;
%this is upper right term of matrix exponental eq

%now to do rodriguez on skew(w)
Sw=skew(w);
expwskew=eye(3)+sin(theta(i))*Sw+(1-cos(theta(i))*(Sw*Sw));

output(:,:,i)=[expwskew UpperRTerm;0 0 0 1];
end

end
