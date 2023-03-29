%imputs jacobian
%outputs liniar and angular elipsoid volumes

function [Lvolume, Avolume]=J_ellipsoid_volume(Jacobian)

LEvector=ellipsoid_plot_linear(Jacobian);
AEvector=ellipsoid_plot_angular(Jacobian);

Lvolume=LEvector(1)*LEvector(2)*LEvector(3);
Avolume=AEvector(1)*AEvector(2)*AEvector(3);

end