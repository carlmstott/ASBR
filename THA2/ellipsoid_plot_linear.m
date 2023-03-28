%This function was written by carl stott on 3/15
%Brief: takes in a jacobian and returns a 4x3 matrix where top row is
%magnatide of principal axis of the ellipsoid and the bottom 3 rows are the
%directions of the axis's of the manuverability elipsoid in columns. ex,
%column 1 of the output will have a scaler magnitude on top and direction
%of axis in the bottom 3 spots
%Params: jacobian
%
%
%Returns: 3,2 matrix where each row is the respective direction and 
% magnitude of the principal axis of the liniar manipuability elipsoid.
function elipsoidDimentions=ellipsoid_plot_linear(jacobian)

J_v=jacobian(4:6,:); %refrence: W8L21S7
J_vt=transpose(J_v);

A_v=J_v*J_vt; %if everything else is correct this should be a 3x3

[V,D]=eig(A_v); %"produces a diagonal matrix D of eigenvalues and 
        %a full matrix V whose columns are the corresponding eigenvectors  
        %so that A*V = V*D." -matlab tooltip

elipsoidDimentions.eigenVectors = V;
elipsoidDimentions.eigenValues = transpose(diag(D));



Evector=[sqrt(elipsoidDimentions.eigenValues(1));
    sqrt(elipsoidDimentions.eigenValues(2)); 
    sqrt(elipsoidDimentions.eigenValues(3))];


ElipsoidDim = V*Evector;


 %plot this
  ellipsoid(0,0,0, sqrt(elipsoidDimentions.eigenValues(1)), ...
      sqrt(elipsoidDimentions.eigenValues(2)), sqrt(elipsoidDimentions.eigenValues(3)))
  axis equal
  hold


% ellipsoid(0,0,0, ElipsoidDim(1), sqrt(ElipsoidDim(2)), ElipsoidDim(3))

% hold on
% quiver3(0,0,0,sqrt(elipsoidDimentions.eigenValues(1))*elipsoidDimentions.eigenVectors(1,1),sqrt(elipsoidDimentions.eigenValues(1))*elipsoidDimentions.eigenVectors(2,1),sqrt(elipsoidDimentions.eigenValues(1))*elipsoidDimentions.eigenVectors(3,1))
% quiver3(0,0,0,sqrt(elipsoidDimentions.eigenValues(2))*elipsoidDimentions.eigenVectors(1,2),sqrt(elipsoidDimentions.eigenValues(2))*elipsoidDimentions.eigenVectors(2,2),sqrt(elipsoidDimentions.eigenValues(2))*elipsoidDimentions.eigenVectors(3,2))
% quiver3(0,0,0,sqrt(elipsoidDimentions.eigenValues(3))*elipsoidDimentions.eigenVectors(1,3),sqrt(elipsoidDimentions.eigenValues(3))*elipsoidDimentions.eigenVectors(2,3),sqrt(elipsoidDimentions.eigenValues(3))*elipsoidDimentions.eigenVectors(3,3))
% 
% quiver3(0,0,0,sqrt(elipsoidDimentions.eigenValues(1))*-elipsoidDimentions.eigenVectors(1,1),sqrt(elipsoidDimentions.eigenValues(1))*-elipsoidDimentions.eigenVectors(2,1),sqrt(elipsoidDimentions.eigenValues(1))*-elipsoidDimentions.eigenVectors(3,1))
% quiver3(0,0,0,sqrt(elipsoidDimentions.eigenValues(2))*-elipsoidDimentions.eigenVectors(1,2),sqrt(elipsoidDimentions.eigenValues(2))*-elipsoidDimentions.eigenVectors(2,2),sqrt(elipsoidDimentions.eigenValues(2))*-elipsoidDimentions.eigenVectors(3,2))
% quiver3(0,0,0,sqrt(elipsoidDimentions.eigenValues(3))*-elipsoidDimentions.eigenVectors(1,3),sqrt(elipsoidDimentions.eigenValues(3))*-elipsoidDimentions.eigenVectors(2,3),sqrt(elipsoidDimentions.eigenValues(3))*-elipsoidDimentions.eigenVectors(3,3))


end

