function [ZYZ,RPY]=ZYZ_and_roll_pitch_yaw(R)

%this function takes a 3x3 rotation matrix R contained in SO(3) and returns
%the ZYZ and roll pitch yaw (ZXY) representations. the output RPY refers to
%the ZYZ representation.

%this function was written by Carl Stott on 2/5/2023.

% 1. check to make sure that R is contained by SO(3).
% to do that, make sure dimentions of R is 3x3 check that det(R)=1, and
% that Rtranspose(R)=I. the use of the any() function reduces a matrix
% or vector of logical values into a single logical, which is what the
% boolian operators need.

if (det(R)~=1 || any(any(R*transpose(R)~=eye(3))) || any(size(R)~=([3,3])))
    error('make sure R is part of SO(3)')
end

%now that I'm sure we are dealing with a matrix in SO(3), I can
% define theta, phi, and psi for my ZYZ orientation


% choosing theta between (0,pi), week 3 lecture 1 slide 8
phi= atan2(R(2,3),R(1,3));
theta= atan2(-sqrt((R(1,3)^2)+(R(2,3)^2)),R(3,3));
psi= atan2(R(3,2),-R(3,1));

if(sin(theta)==0) 
    %this presents a singularity in the ZYZ representation,
    %sin(theta)=0 means theta is zero or pi, and that R is I
    ZYZ='singularity';
else
    ZYZ=[phi;theta;psi];
end

%now I will redefine my phi, theta, and psi for my ZXY orientation W3L1
%slide 9. Using -(3,1) to find theta becasue (3,1)=-sin(theta)
if ((pi/2) > asin(-R(3,1))) && (asin(-R(3,1)) > (-pi/2))
    %the above line checks to see if theta is between -pi/2 and pi/2,
    %theta is found by examining R(3,1) as shown in slide 10 of the
    %w3L1 slide 10
    phi=atan2(R(2,1),R(1,1));
    theta=atan2(-R(3,1),sqrt((R(3,2)^2)+(R(3,3)^2)));
    psi=atan2(R(3,2),R(3,3));

    RPY=[phi;theta;psi];

elseif ((pi/2) < asin(-R(3,1))) && (asin(-R(3,1) < ((3*pi)/2)))
    %the above line checks to see if theta is between pi/2 and
    % (3*pi)/2, theta is found by examining R(3,1) as shown in slide 10 of the
    %week 3 lecture 1 slideshow.
    phi=atan2(-R(2,1),-R(1,1));
    theta=atan2(-R(3,1),-sqrt((R(3,2)^2)+(R(3,3)^2)));
    psi=atan2(-R(3,2),-R(3,3));

    RPY=[phi;theta;psi];
else
    %should only enter this "else" case when theta is +/- pi/2, which
    %presents a singularity for a ZYX rotation.
    RPY="singularity";

end

end

