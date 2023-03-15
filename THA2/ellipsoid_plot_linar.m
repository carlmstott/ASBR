%This function was written by carl stott on 3/15
%Brief: takes in a jacobian and returns a 4,3 matrix where top row is
%magnatide of principal axis of the ellipsoid and the bottom 3 rows are the
%directions of the axis's of the manuverability elipsoid.
%Params: jacobian
%
%
%Returns: 3,2 matrix where each row is the respective direction and 
% magnitude of the principal axis of the liniar manipuability elipsoid.
function elipsoidDimentions=ellipsoid_plot_linar(jacobian)

J_v=jacobian(4:6,:); %refrence: W8L21S7
J_vt=transpose(J_v);

A_v=J_v*J_vt; %if everything else is correct this should be a 3x3

[V,D]=eig(A_v); %"produces a diagonal matrix D of eigenvalues and 
        %a full matrix V whose columns are the corresponding eigenvectors  
        %so that A*V = V*D." -matlab tooltip


elipsoidDimentions=[D(1,1), D(2,2), D(3,3);
                    V(:,1), V(:,2), V(:,3)];
