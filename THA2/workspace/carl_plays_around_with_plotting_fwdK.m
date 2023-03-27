%carl tests plotting fwdK
clear all;
close all;


M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
S = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
B=S;
jointAngles =[7; 2*pi; -5*pi];

[robot, err] = defineRobot(M, S, B);

[T, jointToJointTransforms] = FK_body(robot,jointAngles)