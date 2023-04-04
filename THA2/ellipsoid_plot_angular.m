%This function was written by carl stott on 3/15
%Brief: takes in a jacobian and returns a 4,3 matrix where top row is
%magnatide of principal axis of the ellipsoid and the bottom 3 rows are the
%directions of the axis's of the manuverability elipsoid in columns. ex,
%column 1 of the output will have a scaler magnitude on top and direction
%of axis in the bottom 3 spots
%Params: jacobian
%
%
%Returns: 3,2 matrix where each row is the respective direction and 
% magnitude of the principal axis of the liniar manipuability elipsoid.

function Evector=ellipsoid_plot_angular(jacobian)

J_a=jacobian(1:3,:); %refrence: W8L21S7

J_at=transpose(J_a);


A_v=J_a*J_at; %if everything else is correct this should be a 3x3

[V,D]=eig(A_v); %"produces a diagonal matrix D of eigenvalues and 
        %a full matrix V whose columns are the corresponding eigenvectors  
        %so that A*V = V*D." -matlab tooltip

elipsoidDimentions.eigenVectors = V;
elipsoidDimentions.eigenValues = round(transpose(diag(D)), 4); % round to 4 places after the decimal



Evector=[sqrt(elipsoidDimentions.eigenValues(1));
    sqrt(elipsoidDimentions.eigenValues(2)); 
    sqrt(elipsoidDimentions.eigenValues(3))];


ElipsoidDim = V*Evector;

ElipsoidDim=abs(ElipsoidDim);

figure;
ellipsoid(0,0,0, double(ElipsoidDim(1)), double(ElipsoidDim(2)), double(ElipsoidDim(3)))
axis equal
title("Manipulability ellipsoid - orientation")
xlabel('x')
ylabel('y')
zlabel('z')
