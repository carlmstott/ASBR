function [b_tip, b_post] = pivotCalibration (T_k)

A = [];
B = [];
for i = 1:size(T_k, 3)
    
    % extract rotation and translation
    R_i = T_k(1:3, 1:3, i);
    p_i = T_k(1:3, 4, i);

    % for A and B matrices
    A_i = [R_i, -eye(3, 3)];
    B_i = -p_i;
    A = [A; A_i]
    B = [B; B_i]

end

% solve AX = B
X = mldivide(A, B);

% extract probe parameters
b_tip = X(1:3);
b_post = X(4:6);
end