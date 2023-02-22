function [Q]=quaternion_code(R)
%this function takes a 3x3 rotation matrix R contained in SO(3) and returns
%the unit quarturnian.

%this function was written by Carl Stott on 2/5/2023.
% NOTE YOU NEED TO BE ABLE TO CALL THE SGN() FUNCTION FOR THIS TO WORK. The
% sgn() function is the same as matlab's sign() function but where sign()
% returns a 0, sgn() returns a 1.

% 1. check to make sure that R is contained by SO(3).
    % to do that, make sure dimentions of R is 3x3 check that det(R)=1, and 
    % that Rtranspose(R)=I. the use of the any() function reduces a matrix
    % or vector of logical values into a single logical, which is what the
    % or operator needs.

if (det(R)~=1 || any(any(R*transpose(R)~=eye(3))) || any(size(R)~=([3,3])))
error('make sure R is part of SO(3)')

else %now that R is confirmed to be in SO(3), we can create our 
     %quarturnian, used w3l1 slide 14 as reference
     Q(1,1)=.5*sqrt(R(1,1)+R(2,2)+R(3,3)+1);
     Q(2,1)=.5*(sgn(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1));
     Q(3,1)=.5*(sgn(R(1,3)-R(3,1))*sqrt(R(2,2)-R(3,3)-R(1,1)+1));
     Q(4,1)=.5*(sgn(R(2,1)-R(1,2))*sqrt(R(3,3)-R(1,1)-R(2,2)+1));
end
