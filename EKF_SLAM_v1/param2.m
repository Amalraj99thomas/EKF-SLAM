%%%%%%%%%%%%%%%%%%
%% Author: Anuraga Sankepally 
% Created on - 4 dec 2021
% edited on- 
% version 1 
clc; clear all; close all

%% Landmark data 
P.NumLm = 3;
P.LmX = [1.4 3 5];
P.LmY = [2 6 8];
%% initial states 

%P.totstates = 3 + 2*P.NumLm;
P.init = zeros(3 ,1);
P.NumOutputs=2;
P.radius= 7; 
P.SimTime = 300;

%% noise parameters 
P.spr = 0.5;
P.spv = 0.2;
P.spw = 0.01;

