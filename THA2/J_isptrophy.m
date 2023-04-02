%this function takes in the jacobian and and calcualtes the isotropy from
% each of the 2 ellipsoids

function [Liso, Aiso]=J_isptrophy(Jacobian)

LEvector=ellipsoid_plot_linear(Jacobian);
AEvector=ellipsoid_plot_angular(Jacobian);

Liso = sqrt(max(LEvector)/min(LEvector));
Aiso = sqrt(max(AEvector)/min(AEvector));

end