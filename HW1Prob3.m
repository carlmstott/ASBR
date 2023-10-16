function [Turn, QuarterTurn, HalfTurn, ThreeQuaterTurn]=HW1Prob3(Tas,q,s,h,theta)

%im going to assume that my user isnt going to feed me garbage, so no
%conditions to catch incorrect inpuputs needed, also my other functions
%should catch them anyway.
%this code was written by Carl Stott on 2/22/2022

%vataibles I need in order to compute [S](theta), I need w vector, I need v
%vector, and I need theta (givin). w is the axis of rotation, and that
% is givin to us. But it is called s. so my first step is to make [w],
% I used a function I wrote called the skew function(included in
% assignment)


%now I need to calcualte V
