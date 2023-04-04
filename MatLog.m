function Twist=MatLog(TMatrix)

Tmatrix=se3(TMatrix);

if rotm(Tmatrix)==eye(3)
    w=[0;0;0];
    v=trvec(Tmatrix);
    v=transpose(v)
else
 [theta,omega]=axis_angle_code(rotm(Tmatrix));

w=omega;

GinvTheta=(1/theta)*eye(3)-(1/2)*w+((1/theta)-(1/2)*(cot(theta/2)))*w*w;

v=GinvTheta*transpose(trvec(Tmatrix));

w=[-w(2,3);w(1,3);-w(1,2)]; %converting from skewsym back to a vector

end
Twist=[w;v];
end
