% A and B are column vectors i.e. A = [x1 y1 z1; 
                                     % x2 y2 z2;
                                     % .
                                     % .
                                     %         ]
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

err = det(R);
if(err ~= 1)
    T = 0;
    err = -1;
    return
end

p = transpose(B_centroid) - R * transpose(A_centroid);

T = [R p; 0 0 0 1] ;


end