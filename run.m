clear all; clc
close all;
addpath('./util');
addpath('./output');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015a');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015aom');


A = [-0.5 0; 0.1 -0.2];
B = eye(2);
X0_poly = [1 2; 1.5 2; 1.5 2.5; 1 2.5];
avoid_x = [0.5 0.5 1 1];
avoid_y = [-0.4 -0.8 -0.8 -0.4];
goal_x= [0.95 1.05 1.05 0.95];
goal_y= [0.98 0.98 1.02 1.02];

rsdp(A, B, X0_poly, avoid_x, avoid_y, goal_x, goal_y)