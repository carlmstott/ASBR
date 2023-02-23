function Wskew=skew(w)
%takes a 3 length colum vector, spits out skew sym matrix of that
%written by carl stott on 2/22
Wskew=[0,-w(3),w(2);
     w(3), 0, -w(1);
     -w(2), w(1), 0];
end
