function [R]=axis_angle_to_R(w,theta)

%this function takes an axis and an angle representing a rotation and
%return the rotation matrix representing that rotation.
%this fuction was written by Carl Stott on 2/20/2022

%checking to make sure that my user isnt feeding me theta=0 and 
%checking to make sure w is a valid axis
if(theta==0 || any(size(w)~=([3,1])) || norm(w)==1)
    error('make sure theta and w are valid entrys')
end

%now that I know my user isnt feeding me garbage, I can compute R.
%source: Modern Robotics and Control (Sprong, Hutchinson, Vidyasagar)pg 58,
% eq 2.43
Vtheta=1-cos(theta);
x=w(1);
y=w(2);
z=w(3);
%the above lines make R calulation easier to read.
R=[x*x*Vtheta+cos(theta),   x*y*Vtheta-z*sin(theta), x*y*Vtheta+y*sin(theta);
   x*y*Vtheta+z*sin(theta), y*y*Vtheta+cos(theta),   y*z*Vtheta-x*sin(theta);
   x*z*Vtheta-y*sin(theta), y*z*Vtheta+x*sin(theta), z*z*Vtheta+cos(theta)];
end
