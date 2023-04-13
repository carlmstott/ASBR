%This function was written by carl stott on 3/15
%Brief: takes in a jacobian and returns a 4x3 matrix where top row is
%magnatide of principal axis of the ellipsoid and the bottom 3 rows are the
%directions of the axis's of the manuverability elipsoid in columns. ex,
%column 1 of the output will have a scaler magnitude on top and direction
%of axis in the bottom 3 spots
%Params: jacobian, doWePlot. if plot=1, then we are plotting

function Evector=ellipsoid_plot_linear(jacobian,doWePlot)

J_v=jacobian(4:6,:); %refrence: W8L21S7
J_vt=transpose(J_v);

A_v=J_v*J_vt; %if everything else is correct this should be a 3x3

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

if doWePlot == 1
%figure
subplot(2,2,1);
ellipsoid(0,0,0, double(ElipsoidDim(1)), double(ElipsoidDim(2)), double(ElipsoidDim(3)))
axis equal
title("Manipulability ellipsoid - translation")
xlabel('x')
ylabel('y')
zlabel('z')
end

end

