% lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calbody.txt');
lines = readlines('C:\Users\bhara\Documents\GitHub\ASBR\THA3\data\HW3-PA1\pa1-debug-a-calreadings.txt');

chars_Str = convertStringsToChars(lines(1));
C = regexp(chars_Str, ',', 'split');
C = str2double(C);

lines(1) = [];

if contains(chars_Str, "calbody")
    N_D = C(1);
    N_A = C(2);
    N_C = C(3);

    d_i=zeros(1,3);
    for i=1:N_D
        d_i(i, :) = str2num(lines(i));
    end

    a_i=zeros(1,3);
    iter = 1;
    for i = (N_D + 1) : (N_D + 1 + N_A - 1)
        a_i(iter, :) = str2num(lines(i));
        iter = iter + 1;
    end

    c_i=zeros(1,3);
    iter = 1;
    for i = (N_D + 1 + N_A) : (N_D + 1 + N_A + N_C - 1)
        c_i(iter, :) = str2num(lines(i));
        iter = iter + 1;
    end

elseif contains(chars_Str, "calreadings")
    N_D = C(1);
    N_A = C(2);
    N_C = C(3);
    N_frames = C(4);

    lines_per_frame = N_D + N_A + N_C;

    for i = 1 : N_frames
        for j = 1 : N_D
            (i-1)*27 + j
            D_i(j, :, i) = str2num(lines((i-1)*lines_per_frame + j))
        end
        for k = 1 : N_A
            (i-1)*27 + k + 1
            A_i(k, :, i) = str2num(lines((i-1)*lines_per_frame + k + N_D))
        end
        for l = 1 : N_C
            (i-1)*27 + l + 2
            C_i(l, :, i) = str2num(lines((i-1)*lines_per_frame + l + N_D + N_A))
        end
    end

elseif contains(chars_Str, "empivot")

elseif contains(chars_Str, "optpivot")


end
