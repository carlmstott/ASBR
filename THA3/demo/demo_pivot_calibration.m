clc; clear;
% lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-b-empivot.txt');
% readTxt
% lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-b-optpivot.txt');
% readTxt
% format long g

lines = readlines('C:\Users\carlm\OneDrive\Desktop\spring semester 2023\github\ASBR\THA3\data\HW3-PA1\pa1-debug-b-empivot.txt');
readTxt
lines = readlines('C:\Users\carlm\OneDrive\Desktop\spring semester 2023\github\ASBR\THA3\data\HW3-PA1\pa1-debug-b-optpivot.txt');
readTxt
format long g

% use the first frame to define a local probe coordinate system
% calculate centroid
G_centroid = mean(G_i(:, :, 1));
figure; hold on;
% compute g_i
for i = 1:N_frames
    g_i(:, :, i) = G_i(:, :, i) - G_centroid;

    [T, err] = least_squares_registration(g_i(:, :, i), G_i(:,:,i));

    plotter(g_i(:, :, i), G_i(:,:,i), T)

    T_tall(:, :, i) = T;
    pause(2)

    clf('reset')

end

[b_tip, b_post] = pivotCalibration(T_tall)
