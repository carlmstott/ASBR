%This function was written by carl stott on 3/15
%Brief: takes in a jacobian and returns a 3,2 matrix where each row is the 
% respective direction and magnitude of the principal axis of the liniar
% manipuability elipsoid.
%Params: jacobian
%
%
%Returns: 3,2 matrix where each row is the respective direction and 
% magnitude of the principal axis of the liniar manipuability elipsoid.
function elipsoidDimentions=ellipsoid_plot_linar(jacobian)

J_v=jacobian()