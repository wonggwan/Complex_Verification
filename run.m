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

room1_goal_x= [1.95 2.05 2.05 1.95]; % successful case
room1_goal_y= [0.98 0.98 1.02 1.02];

disp("sub-task1: room 1")
controller_1 = './output/room1.mat';
[is_satisfied_1, FRS_V_bd_1] = rsdp(A, B, X0_poly, avoid_x, avoid_y, room1_goal_x, room1_goal_y, controller_1);
if is_satisfied_1 == 1
    disp("sub-task2: room 2")
    X0_poly_2 = FRS_V_bd_1;
    room2_goal_x = [2.45 2.55 2.55 2.45]; % successful case
    room2_goal_y = [2.1 2.1 2.3 2.3];
    % room2_goal_x = [3.45, 3.55, 3.55, 3.45];
    % room2_goal_y =[3.1, 3.1, 3.3, 3.3];
    controller_2 = './output/room2.mat';
    [is_satisfied_2, FRS_V_bd_2] = rsdp(A, B, X0_poly_2, avoid_x, avoid_y, room2_goal_x, room2_goal_y, controller_2);
    if is_satisfied_2 == 1
        disp("sub-task3: room 4")
        X0_poly_3 = FRS_V_bd_2;
        room4_goal_x = [-1.01, -0.98, -0.98, -1.01];
        room4_goal_y = [-1.01, -1.01, -0.98, -0.98];
        controller_4 = './output/room4.mat';
        [is_satisfied_3, FRS_V_bd_3] = rsdp(A, B, X0_poly_3, avoid_x, avoid_y, room4_goal_x, room4_goal_y, controller_4);
        if is_satisfied_3 == 1
            disp("Verfication succeeded")
        else
            disp("Verfication failed")
        end
    else
        disp("sub-task4: room 3")
        X0_poly_4 = FRS_V_bd_2;
        room3_goal_x = [1.18 1.25 1.25 1.18];
        room3_goal_y = [1.38 1.38 1.42 1.42];
        controller_3 = './output/room3.mat';
        [is_satisfied_4, FRS_V_bd_4] = rsdp(A, B, X0_poly_4, avoid_x, avoid_y, room3_goal_x, room3_goal_y, controller_3);
        if is_satisfied_4
            disp("sub-task5: room 4")
            X0_poly_5 = FRS_V_bd_4;
            room4_goal_x = [-1.01, -0.98, -0.98, -1.01];
            room4_goal_y = [-1.01, -1.01, -0.98, -0.98];
            controller_4 = './output/room4.mat';
            [is_satisfied_5, FRS_V_bd_5] = rsdp(A, B, X0_poly_5, avoid_x, avoid_y, room4_goal_x, room4_goal_y, controller_4);
            if is_satisfied_5
                disp("Verfication succeeded")
            else
                disp("Verfication failed")
            end
        else
            disp("Verfication failed")
        end
    end
else
    disp('Verfication failed')
end