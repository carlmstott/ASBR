clear; clc;

M = [1 0 0 3;0 1 0 0;0 0 1 0;0 0 0 1];

S(1).w = [0; 0; 1];
S(1).v = [0;0;0];

S(2).w = [0; 0; 0];
S(2).v = [1;0;0];

S(3).w = [0; 0; 1];
S(3).v = [0;-1;0];

S(4).w = [0; -1; 0];
S(4).v = [-1;0;-1];

S(5).w = [0; 0.707; 0.707];
S(5).v = [0.707;-1.414;1.414];

B(1).w = [0; 0; 1];
B(1).v = [0;3;0];

B(2).w = [0; 0; 0];
B(2).v = [1;0;0];

B(3).w = [0; 0; 1];
B(3).v = [0;2;0];

B(4).w = [0; -1; 0];
B(4).v = [-1;0;2];

B(5).w = [0; 0.707; 0.707];
B(5).v = [0.707;0.707;-0.707];

robot = defineRobot(M, S, B)