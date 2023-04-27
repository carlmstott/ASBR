% brief: this function registers (finds a transfrom from point cloud B to
% point cloud A)
% params:
% A: point cloud A column vector i.e. A = [x1 y1 z1; 
                                     % x2 y2 z2;
                                     % .
                                     % .
                                     %         ]
% B: point cloud B column vector i.e. B = [x1 y1 z1; 
                                     % x2 y2 z2;
                                     % .
                                     % .
                                     %         ]
% returns:
% T: transfromation matrix from B to A
% err: determinant of the rotation matrix. This is not necessarily the
% determinant of the final rotation matrix outputed from this algorithm

function [T, err] = least_squares_registration(A, B)

% calculate centroids
A_centroid = mean(A);
B_centroid = mean(B);

% calculate vectors from centroids
A_prime = A - A_centroid; % A_tilda = ai - mean_A
B_prime = B - B_centroid; % B_tilda = bi - mean_B

H = zeros(3,3);

for i = 1: size(A, 1)
    H = H + A_prime(i, :) * transpose(B_prime(i, :));
end

[U, S, V] = svd(H);

R = V * transpose(U);

% check and fix det(R) = -1 by using its reflection. R' = V'transpose(U)
% where V' = [v1 v2 -v3]
err = det(R);
if abs(err - 1) > 1e-6
    V(:, 3)=V(:, 3)*(-1);
    R = V*transpose(U);
end

p = transpose(B_centroid) - R * transpose(A_centroid);

T = [R p; 0 0 0 1] ;


end