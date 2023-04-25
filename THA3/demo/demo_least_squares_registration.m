clc; clear;
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-c-calreadings.txt');
readTxt
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-c-calbody.txt');
readTxt
format long g
for iter = 1:N_frames
    [T, err] = least_squares_registration(d_i, D_i(:,:,iter))


    p = plot3(d_i(:, 1), d_i(:, 2), d_i(:, 3))
    p.Marker = '.'
    p.LineStyle = "none"

    hold on
    axis equal

    p_D = plot3(D_i(:,1,2), D_i(:,2,2), D_i(:,3,2))
    p_D.Marker = '.'
    p_D.LineStyle = "none"
    p_D.Color = 'red'

    for i = 1 : size(d_i, 1)
        d = transpose(d_i(i, :));
        d(4) = 1;
        corrected_intermediate = transpose(T * d);
        corrected_intermediate(:, 4) = [];
        corrected(i, :, iter) = corrected_intermediate
    end
    
    p_cor = plot3(corrected(:,1, iter), corrected(:,2, iter), corrected(:,3, iter))
    p_cor.Marker = 'o'
    p_cor.LineStyle = "none"
    p_cor.Color = 'yellow'

    close all
end