function adjoint= Adj(x)

%this function takes in the 4x4 matrix exponental form of a twist vector
% x and returns its adjoint representation
R=x(1:3,1:3); %this is the top 3x3 of x
P=x(1:3,4); %this is the first 3 rows of the 4th column, aka the displacement 
           

adjoint=[R, zeros(3); skew(P)*R, R];
