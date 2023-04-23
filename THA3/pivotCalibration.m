function [b_tip, b_post] = pivotCalibration (T_k)
R_k = T_K(1:3, 1:3);
p_k = T_k(1:3, 4);

A = [R_k, -eye(3,3)];
B = -p_k;

X = mldivid (A, B);

b_tip = X(1:3);
b_post = X(4:6);
end