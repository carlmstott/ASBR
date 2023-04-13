%this function takes in the jacobian and and calcualtes the isotropy from
% each of the 2 ellipsoids

%reference, W8L1S8

function [Liso, Aiso]=J_isptrophy(Jacobian)

LEvector=ellipsoid_plot_linear(Jacobian,0);
AEvector=ellipsoid_plot_angular(Jacobian,0);

Liso = sqrt(max(LEvector)/min(LEvector));
Aiso = sqrt(max(AEvector)/min(AEvector));

end
