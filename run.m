clear all; clc
close all;
addpath('./util');
addpath('./output');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015a');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015aom');


A = [-0.5 0; 0.1 -0.2];
B = eye(2);
X0_poly = [1 2; 1.5 2; 1.5 2.5; 1 2.5];
avoid_x = [1 1.2 1.2 1];
avoid_y = [1 1 1.2 1.2];

room1_goal_x= [1.95 2.05 2.05 1.95];
room1_goal_y= [0.98 0.98 1.02 1.02];

% room 1
controller_1 = './output/room1.mat';
[is_satisfied_1, FRS_V_bd_1] = rsdp(A, B, X0_poly, avoid_x, avoid_y, room1_goal_x, room1_goal_y, controller_1);
FRS_V_bd_1
clf;
if is_satisfied_1
    % room 2
    X0_poly_2 = FRS_V_bd_1;
    room2_goal_x = [2.45 2.55 2.55 2.45];
    room2_goal_y = [2.1 2.1 2.3 2.3];
    controller_2 = './output/room2s.mat';
    [is_satisfied_2, FRS_V_bd_2] = rsdp(A, B, X0_poly_2, avoid_x, avoid_y, room2_goal_x, room2_goal_y, controller_2);
    FRS_V_bd_2
else
    disp('Verfication failed')
end