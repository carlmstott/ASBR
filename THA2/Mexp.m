function Mexp=(robot,theta)

theta = sym('theta', [robot.numJoints 1]);

Ans=eyes(2);
%theta is the angle being applied to the twist vector

w=robot.s(1:3,1); %first three rows of a twist vector is w
v=robot.s(4:6,1);

Ans(2,2)=eye(3)*theta+(1-cos(theta)*w+(theta-sin(theta)))(w*w)
