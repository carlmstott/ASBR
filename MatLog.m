function Twist=MatLog(TMatrix)

Tmatrix=se3(TMatrix);

if rotm(Tmatrix)==eye(3)
    w=[0;0;0];
    v=trvec(Tmatrix);
    v=transpose(v);
else
 [theta,skewW]=axis_angle_code(rotm(Tmatrix));


w=[-skewW(2,3);skewW(1,3);-skewW(1,2)]; %converting from skewsym back to a vector

GinvTheta=((1/theta)*eye(3)) - ((1/2)*skew(w)) + ( (1/theta) - (1/2)*(cot(theta/2)) )*skew(w)*skew(w);

v=GinvTheta*transpose(trvec(Tmatrix));


end
Twist=[w;v];
end
