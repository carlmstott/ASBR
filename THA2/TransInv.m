function invT = TransInv(T)
R = T(1:3, 1:3);
p = T(1:3, 4);
invT = [transpose(R), -transpose(R) * p; 0, 0, 0, 1];
end