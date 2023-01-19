%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%plot file %%%%%%%%%%%%%%
%%
function plot_function(P) 
clear all;
clc; close all;

load Plotfile.mat
t       = dat.Time;
veh_st  = dat.Data;
f       = 0;

% true states 
NN      = 0;
x       = veh_st(:,NN+1);
y       = veh_st(:,NN+2);
psi     = veh_st(:,NN+3);
time     = veh_st(:,NN+4);

% NN      = NN+3;
lmx1    = 1;
lmy1    = 2;
lmx2    = 3;
lmy2    = 5;
lmx3    = 5;
lmy3    = 6;

% state estimates
NN      = NN + 4;
pxt     = veh_st(:,NN+1);
pyt     = veh_st(:,NN+2);
psit    = veh_st(:,NN+3);
%landmark estimates 
lmx1t =  veh_st(:,NN+4);
lmy1t =  veh_st(:,NN+5);
lmx2t =  veh_st(:,NN+6);
lmy2t =  veh_st(:,NN+7);

% covariance 
NN      = NN + 3;

px_cov  = veh_st(:,NN+1);
py_cov  = veh_st(:,NN+2);
psi_cov = veh_st(:,NN+3);
lmx1_cov = veh_st(:,NN+4);
lmy1_cov = veh_st(:,NN+5);

wpsit = wrapToPi(psit);
wpsi  = wrapToPi(psi);


% 
%NN      = NN + 3;

% v       = veh_st(:,NN+1);
% w       = veh_st(:,NN+2);


figure(f+1)
   grid_min     = 0;
   grid_max     = 10;
   NumLm        = 3;
   Lmx          = grid_min + (grid_max-grid_min)*rand(NumLm,1);
   Lmy          = grid_min + (grid_max-grid_min)*rand(NumLm,1);
 
   
   plot(Lmx,Lmy,'r*');
   hold on;
   plot(lmx1t, lmy1t, 'b*');
   hold on;
   plot(x,y,'k','LineWidth',2);
   hold on;
   plot(pxt,pyt,'r--','LineWidth',2);
   set(gcf,'Units','centimeters','Position',[10,0,20,15])
   set(gca,'FontSize',12);
   legend('Est states','true states');
%% True vs estimated states 
 
figure (f+2)
set(gcf,'Units','centimeters','Position',[10,0,20,15])
subplot(3,1,1)
plot(t,pxt,'r--','LineWidth',2);
hold on;
plot(t,x,'k','LineWidth',1);
legend('true','estimated');
 xlabel('time (s)','FontWeight','bold')
 ylabel('px_{10} (m)','FontWeight','bold')
 set(gca,'FontSize',12);
 
 
 subplot(3,1,2)
 plot(t,pyt,'r--','LineWidth',2);
 hold on 
 plot(t,y,'k','LineWidth',1);
 xlabel('time (s)','FontWeight','bold')
 ylabel('py_{10} (m)','FontWeight','bold')
 set(gca,'FontSize',12);
 
 subplot(3,1,3)
    
%     plot(t,(pi/180)*psit,'r--','LineWidth',2);
    plot(t,wpsit,'r--','LineWidth',2);
    hold on;
%     plot(t,(pi/180)*psi,'k','LineWidth',1);
    plot(t,wpsi,'k','LineWidth',1);
    %xticks(0:20:200);
    %yticks(-180:60:180);
    xlabel('time (s)','FontWeight','bold')
    ylabel('\psi^{0}_{10}','FontWeight','bold')
    set(gca,'FontSize',12);
  %% error covariance plots 
  figure(f+3)
  
    subplot(4,1,1)
    set(gcf,'Units','centimeters','Position',[10,0,20,15])
    plot(t,pxt-x,t,3*px_cov.^0.5,'r--',t,-3*px_cov.^0.5,'r--','LineWidth',2);
    xlabel('t (s)','FontWeight','bold')
    ylabel('px_{{10}_{err}} (m)','FontWeight','bold')
    legend('Error','3 \sigma')
    set(gca,'FontSize',12);
    
    subplot(4,1,2)
    plot(t,pyt-y,t,3*py_cov.^0.5,'r--',t,-3*py_cov.^0.5,'r--','LineWidth',2);
    xlabel('t (s)','FontWeight','bold')
    ylabel('py_{{10}_{err}} (m)','FontWeight','bold')
    
    set(gca,'FontSize',12);
    
    subplot(4,1,3)
    plot(t,(pi/180)*(psit-psi),t,(pi/180)*3*psi_cov.^0.5,'r--',t,-(pi/180)*3*psi_cov.^0.5,'r--','LineWidth',2);
    xlabel('t (s)', 'FontWeight','bold')
    ylabel('psi_{{10}_{err}} (m)','FontWeight','bold')
    
    subplot(4,1,4)
    plot(t,lmx1t-lmx1,t,3*lmx1_cov.^0.5,'r--',t,-3*lmx1_cov.^0.5,'r--','LineWidth',2);
    xlabel('t (s)', 'FontWeight','bold')
    ylabel('Landmark1_{{10}_{err}} (m)','FontWeight','bold')

  