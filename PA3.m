clear all; clc;

%initialize test values
s_hat=[0;0;1];
q = [0;2;0];
h=2;
theta = pi;
T = [1 0 0 2;0 1 0 0;0 0 1 0;0 0 0 1];
[T1, S1]=screw_func(q, s_hat, h, theta, T)


% brief: calcultaes final configuration T1 = e_s_theta * T at theta/4, theta/2, 3*theta/4, theta
%
% params:
% q: point on screw axis
% s_hat: screw axis
% h: pitch
% theta: total distance traveled along screw axis
% T: transformation matrix
%
% returns:
% T1: e_s_theta * T at theta/4, theta/2, 3*theta/4, theta
% S1: screw axis [[w] v; 0 0]
%

function [T1, S1]=screw_func(q, s_hat, h, theta, T)
configurations = {theta/4, theta/2, 3*theta/4, theta};
    if(h == Inf)
        %If pitch is infinite
        v=[0;0;1];
        for iter = 1:length(configurations)
            e_s_theta(:, :, iter) = [eye(3) v*configurations{iter}; 0 0 0 1];
            T1(:, :, iter) = e_s_theta(:, :, iter)*T;
        end

    S1 = [0 0 0 v(1);0 0 0 v(2);0 0 0 v(3); 0 0 0 0];

    else
        %If pitch is finite
        omega_two_norm = 1;  % week 5 Lecture 2 slide 11
        theta_dot  = omega_two_norm; % angular velocity. % week 5 Lecture 2 slide 8
        omega = [0;0;1]; %#ok<*NASGU> %omega is unity, thus we assume the screw rotates about its axis at uit angular veocity. % week 5 Lecture 2 slide 10
        omega_hat = [0 -1 0; 1 0 0;0 0 0]; % skew symmteric of omega
        v = -cross(s_hat*theta_dot, q)+h*s_hat*theta_dot; % v extracted from twist. % week 5 Lecture 2 slide 7
        
        for iter = 1:length(configurations)
            e_omega_theta = eye(3) + omega_hat*sin(configurations{iter}) + (omega_hat^2)*(1-cos(configurations{iter})); %rodriques' formula
            translation_vector = (eye(3)*configurations{iter} + (1-cos(configurations{iter}*omega_hat + (configurations{iter}-sin(configurations{iter})) * omega_hat^2)))*v; % week 5 Lecture 2 slide 11
            e_s_theta(:, :, iter) = [e_omega_theta translation_vector ; 0 0 0 1]; % week 5 Lecture 2 slide 11
            T1(:, :, iter) = e_s_theta(:, :, iter)*T;
        end
        S1 = [0 -1 0 v(1);1 0 0 v(2);0 0 0 v(3); 0 0 0 0]; % Modern robotics book pg 108  
    end
end



