clc; clear;
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calreadings.txt');
readTxt
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calbody.txt');
readTxt
format long g
[T, err] = least_squares_registration(d_i, D_i(:,:,2))
