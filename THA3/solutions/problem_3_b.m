clc; clear;
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calreadings.txt');
readTxt
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calbody.txt');
readTxt
format long g
for i = 1 : N_frames
    F_A(:, :, i) = least_squares_registration(a_i, A_i(:,:,i));
    figure; hold on
    plotter(a_i, A_i(:,:,i), F_A(:, :, i))

end

F_A