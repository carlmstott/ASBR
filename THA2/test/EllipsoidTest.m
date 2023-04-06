clear all;
close all;
clc;

%ellipsoid plot tests:
KukaRobot 
%we generate a Kuka robot for testing, and we next generate its body
% jacobian
jointAngles= [0;0;0;0;0;0];
jacobian=J_body(robot, jointAngles);

EvectorAngular=ellipsoid_plot_angular(jacobian);
EvectorLiniar=ellipsoid_plot_linear(jacobian);

%Liso is liniar isotrophy
%Aiso is angular isotrophy
[Liso, Aiso]=J_isptrophy(jacobian)

%Lvolume is liniar ellipsoid volume
%Avolume is angular ellipsoid volume
[Lvolume, Avolume]=J_ellipsoid_volume(jacobian)
