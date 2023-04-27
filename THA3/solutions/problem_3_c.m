clc; clear;
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calreadings.txt');
readTxt
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calbody.txt');
readTxt
format long g
c_i(:, 4) = 1;
for i = 1 : N_frames
    F_A(:, :, i) = least_squares_registration(a_i, A_i(:,:,i));
    F_D(:, :, i) = least_squares_registration(d_i, D_i(:,:,i));

    for j = 1 : size(c_i, 1)
        C_i_expected(j, :, i) = transpose(TransInv(F_D(:, :, i)) * F_A(:, :, i) * transpose(c_i(j, :)));
    end

end

C_i_expected(:, 4, :) = []