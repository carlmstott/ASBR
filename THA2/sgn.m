function y=sgn(x)

%this function is a little tweak on matlab's sign() function, but when
%sign() would return zero, this function returns 1.

%this function was written by Carl Stott on 2/5/2023.
if sign(x)==0
    y=1;
else
    y=sign(x);
end