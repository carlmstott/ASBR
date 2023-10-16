function [theta,w,so3mat]=axis_angle_code(R)
%this function takes a 3x3 rotation matrix R contained in SO(3) and returns
%the axis angle representation of that R such that exp((w)^(theta))=R.
%Where w is the unit axis of rotation (||w||=1) and theta is the angle of
%rotation (theta will be between 0 and pi).

%this function was written by Carl Stott on 2/5/2023. 


% 1. check to make sure that R is contained by SO(3).
    % to do that, make sure dimentions of R is 3x3 check that det(R)=1, and 
    % that Rtranspose(R)=I. the use of the any() function reduces a matrix
    % or vector of logical values into a single logical, which is what the
    % or operator needs.
if (det(R)~=1 || any(any(R*transpose(R)~=eye(3))) || any(size(R)~=([3,3])))
    error('make sure R is part of SO(3)')

elseif (isequal(R,eye(3)))
    %2. if R is an ID matrix, return theta=0 and w as undifined
    theta=0;
    w="undefinied";

    %these below statements were written using the matrix logorithem of a
    %rotation matrix, definied in slide 11 of week 2 lecture 2
elseif (trace(R)==-1)
    %3. if trace R is -1, than theta is pi and we are in a singularity case
    theta=pi;
    w=(1/sqrt(2*(1+R(3,3))))*[R(1,3);R(2,3);1+R(3,3)];
    

else %if this line is reached, R is in so(3), and trace(R)/=-1.
    theta=acos(.5*(trace(R)-1));
    w=(1/(2*sin(theta)))*(R-transpose(R));
end
end
