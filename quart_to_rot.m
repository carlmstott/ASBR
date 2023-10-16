function [R]=quart_to_rot(Q)

%this function takes a quaternion and returns a rotation matrix
%this fuction was written by Carl Stott on 2/20/2022

%first lets check to make sure my user is feeding me a 4x1
if any(size(Q)~=([4,1]))
    error('function only accepts 4x1 vectors')
end
%first lets turn our quaternion into a unit quaternion
M=sqrt(Q(2)+Q(3)+Q(4));
%the below lines make forming R easier
Qo=Q(1);
%dividing the vector components of Q give me a unit quaternion
Q1=Q(2)/M;
Q2=Q(3)/M;
Q3=Q(4)/M;

R=[Qo^2+Q1^2-Q2^2-Q3^2, 2*(Q1*Q2-Q0*Q3),   2*(Qo*Q2+Q1*Q3);
   2*(Qo*Q3+Q1*Q2),   Qo^2-Q1^2+Q2^2-Q3^2, 2*(Q2*Q3-Qo*Q1);
   2*(Q1*Q3-Qo*Q2),   2*(Qo*Q1+Q2*Q3),    Qo^2-Q1^2-Q2^2+Q3^2];

end
