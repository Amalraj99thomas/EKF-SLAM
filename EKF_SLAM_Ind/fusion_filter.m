
%%%%%%%%%%%%% Fusion algorithm %%%%%%%%%%%%
function out = fusion_filter(in,P)

%% filter 1 local estimates 
NN =0;

x1 = in(NN +1);
y1 = in(NN+2);
ps1 = in(NN+3);

NN = NN+3;
lx1 = in(NN+1);
ly1 = in(NN+1);

NN = NN + 2;

P1_x1 = in(NN+1);
P2_y1 = in(NN+2);
P3_ps1 = in(NN+3);
P4_lnx1 = in(NN+4);
P5_lny1 = in(NN+4);

%% Filter 2 local estimates 
NN = NN + 1;
x2 = in(NN +1);
y2 = in(NN+2);
ps2 = in(NN+3);

NN = NN + 3;
lx2 = in(NN+1);
ly2 = in(NN+2);

NN = NN + 2;

P1_x2 = in(NN+1);
P2_y2 = in(NN+2);
P3_ps2 = in(NN+3);
P4_lnx2 = in(NN+4);
P5_lny2 = in(NN+4);

%% Filter 3 local estimates 
NN = NN + 1;
x3 = in(NN +1);
y3 = in(NN+2);
ps3 = in(NN+3);

NN = NN + 3;
lx3 = in(NN+1);
ly3 = in(NN+2);
NN = NN +2;

P1_x3 = in(NN+1);
P2_y3 = in(NN+2);
P3_ps3 = in(NN+3);
P4_lnx3 = in(NN+4);
P5_lny3 = in(NN+4);

%% Bayesian fusion 
%% Robot states 

x = 1/(1/x1^2 + 1/x2^2 + 1/x3^2);
y = 1/(1/y1^2 + 1/y2^2 + 1/y3^2);
ps = wrapToPi((1/ps1^2 + 1/ps2^2 + 1/ps3^2));

x_r = [x;y;ps];

%% Landmark states 
x_l = [lx1;ly1;lx2;ly2;lx3;ly3];

%% Covariance States 
P1_x = 1/(1/P1_x1^2 + 1/P1_x2^2 + 1/P1_x3^2);
P2_y = 1/(1/P2_y1^2 + 1/P2_y2^2 + 1/P2_y3^2);
P3_ps = 1/(1/P3_ps1^2 + 1/P3_ps2^2 + 1/P3_ps3^2);

P_r = [P1_x;P2_y;P3_ps];
P_l = [P4_lnx1;P5_lny1;P4_lnx2;P5_lny2;P4_lnx3;P5_lny3];

out = [...
        x_r;...
        x_l;...
        P_r;...
        P_l;...
        ];
end 







