%%%%%%%%%%%%%%%%%%
%% Author: Anuraga Sankepally 
% Created on - 8 dec 2021
% edited on- 
% version 2 
clc; clear all;
close all

%% Landmark data 
P.NumLm = 3;
P.LmX = [1.4 3 5];
P.LmY = [2 6 8];
%% initial states 

P.init = zeros(3 ,1);
P.NumOutputs=2;
P.radius= 7; 
P.SimTime = 300;

%% noise parameters 
P.spr1 = 0.1; % range sensor noise 
P.spr2 = 0.001; % bearing noise
P.spr3 = 0.1; % magnetic compass noise 

P.spv = 0.01;  %IMU data 
P.spw = 0.01; 

%% Flag variable - switch 
% P.sensor_flag = 1;
P.range_flag = 0;
P.bearing_flag = 1;
P.magneto_flag = 0;





