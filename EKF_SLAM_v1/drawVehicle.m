function drawVehicle(in,P)
    pn = in(1);
    pe = in(2);
    psi = in(3);
    
    NN = 3; 
    pn_est = in(NN+1);
    pe_est = in(NN+2);
    psi_est = in(NN+3);
    
    NN = NN + 3;
    
    lnx1 = in(NN+1);
    lny1 = in(NN+2);
    lnx2 = in(NN+3);
    lny2 = in(NN+4);
    lnx3 = in(NN+5);
    iny3 = in(NN+6);
    
    NN = NN + 2*P.N;
    
    t = in(NN);
    
    %persistent vehicle_handle
    persistent true_states; 
    persistent est_states;
    
    persistent traj_true;
    persistent traj_est; 
    
    persistent pn_traj;
    persistent pe_traj;
    
    persistent pn_traj_e;
    persistent pe_traj_e;
   
    
    if t == 0
%         figure(1); clf;
%         %hold on;
%         vehicle_handle = drawFunction(x,y,psi,[]);
        close all
        
        figure(1)
        plot(P.LmY(1,:),P.LmX(1,:),'ks','MarkerSize',10);
        hold on
        axis([-15 15 -15 15]);
        hold on 
        
        true_states = plot(pe, pn, 'r+', 'MarkerSize',20);
        xlabel('East(m)');
        ylabel('North(m)')
        hold on
        
        est_states = plot(pe_est, pn_est, 'bo');
        
        pn_traj_e=pn_est;
        pe_traj_e=pe_est;
        
        pn_traj=pn;
        pe_traj=pe;
        
        traj_true=plot(pe_traj,pn_traj,'b', 'LineWidth',1);
        hold on 
        traj_est=plot(pe_traj_e,pn_traj_e,'r--', 'LineWidth',1);
        
    else
        %drawFunction(x,y,psi,vehicle_handle);
        
        pn_traj_e=[pn_traj_e;pn_est];
        pe_traj_e=[pe_traj_e;pe_est];
        
        pn_traj=[pn_traj;pn];
        pe_traj=[pe_traj;pe];
        set(true_states,'Xdata',pe,'Ydata', pn); 
        set(est_states,'Xdata',pe_est,'Ydata', pn_est);  
        set(traj_true,'Xdata',pe_traj,'Ydata', pn_traj);
        set(traj_est,'Xdata',pe_traj_e,'Ydata', pn_traj_e);
        drawnow;
        
        axis([-15 15 -15 15]);
    end
end

% function handle = drawFunction(x,y,psi,handle)
% 
%     Rot = [...
%         cos(psi) -sin(psi);...
%         sin(psi) cos(psi);...
%         ];
%     
%     cod = [...
%         2 0 0 2;...
%         0 1 -1 0;...
%         ];
%     
%     codn = [x;y]+Rot*cod;
%     X = codn(1,:);
%     Y = codn(2,:);
%     
%     if isempty(handle)
%         handle = fill(X,Y,'r');
%     else
%         set(handle,'XData',X,'YData',Y);
%     end
% end
    