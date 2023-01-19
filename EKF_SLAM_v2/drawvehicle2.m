function drawvehicle1(in,P)
    pn = in(1);
    pe = in(2);
    psi = in(3);
    t = in(4);
    
    NN = 4; 
    pn_est = in(NN+1);
    pe_est = in(NN+2);
    psi_est = in(NN+3);
    
    NN = NN + 3;
    
    lnx1 = in(NN+1);
    lny1 = in(NN+2);
    lnx2 = in(NN+3);
    lny2 = in(NN+4);
    lnx3 = in(NN+5);
    lny3 = in(NN+6);
     
    persistent true_states; 
    persistent est_states;
    
    persistent traj_true;
    persistent traj_est; 
    
    persistent pn_traj;
    persistent pe_traj;
    
    persistent pn_traj_e;
    persistent pe_traj_e;
    
    persistent ln_states1;
    persistent ln_states2;
    persistent ln_states3;
   
    
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
        ln_states1 = plot(lny1, lnx1, 'o', 'MarkerFaceColor', 'b');
        hold on 
        ln_states2 = plot(lny2, lnx2, 'o', 'MarkerFaceColor', 'b');
        hold on
        ln_states3 = plot(lny3, lnx3, 'o', 'MarkerFaceColor', 'b');
        hold on
        legend('actual landmarks','estimated landmarks');
        
        
        true_states = plot(pe, pn, 'r+', 'MarkerSize',20);
        xlabel('East(m)');
        ylabel('North(m)');
        hold on
        
        
        est_states = plot(pe_est, pn_est, 'bo');
        
        pn_traj_e=pn_est;
        pe_traj_e=pe_est;
        
        pn_traj=pn;
        pe_traj=pe;
        
        traj_true=plot(pe_traj,pn_traj,'b', 'LineWidth',1);
        legend('True Landmarks','est Lns');
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

    