%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%EKF for SLAM 
%Author: Anuraga Sankepally 
%Created on: 4-12-2021
%Edited on: 12-8-2021
%Version: 1

function out = Estimator_SLAM(in,P)
%% IMU inputs
    NN = 0;
    v = in(NN+1);
    w = in(NN+2);
%% Range Measurements
    NN = NN+2;
    rho = in(NN+1:NN+P.NumLm);
%% Time
    NN = NN+P.NumLm;
    t = in(NN+1);
%%
    qu = zeros(2,2);
    qu(1,1) = P.spv^2;
    qu(2,2) = P.spw^2;
%%
    Rt = P.spr^2;
%%  Initialize 
    persistent xHat Pt tPr
    
    if t == 0 
        
        xHat = [0;0;0;1.2;2.4;3.5;5.1;5.2;6.3]+0.1*randn(9,1);
        Pt = 0.1*eye(9);
        tPr = t;
    
    else 
            Ts = t - tPr;
            steps = 10;
        for i = 1:1:steps
            NN = 0;
            pxHat = xHat(NN+1);
            pyHat = xHat(NN+2);
            psiHat = xHat(NN+3);
            mhat = xHat(NN+4:NN+9);
            
            pxHat_dot = v*cos(psiHat);
            pyHat_dot = v*sin(psiHat);
            psiHat_dot = w;
            
            ft = [pxHat_dot;pyHat_dot;psiHat_dot;0; 0; 0; 0; 0; 0];
            
            xHat = xHat + (Ts/steps)*ft;
            
            At = [...
                    0 0 -v*sin(psiHat) 0 0 0 0 0 0; ...
                    0 0  v*cos(psiHat) 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                ];
            Bt = [...
                    cos(psiHat) 0;...
                    sin(psiHat) 0;...
                              0 1;...
                              0 0;...
                              0 0;...
                              0 0;...
                              0 0;...
                              0 0;...
                              0 0;...
                  ];
            Qu = Bt*qu*Bt';
            Pt = Pt + (Ts/steps)*(Qu + At*Pt + Pt*At');
        end
%% Measurement Updates
        
        for j = 1:1:P.NumLm
            NN = 0;
            pxHat = xHat(NN+1);
            pyHat = xHat(NN+2);
            psiHat = xHat(NN+3);
            mHat   = xHat(NN+4:NN+9);
            
           
            ct = sqrt((pxHat-P.LmX(j))^2+(pyHat-P.LmY(j))^2);
            rho10 = rho(j);
            
                if j == 1 
                    
                    Ct = [(pxHat-P.LmX(j))/ct (pyHat-P.LmY(j))/ct 0 (P.LmX(j)-pxHat)/ct (P.LmY(j)-pyHat)/ct 0 0 0 0];
            
                elseif j == 2 
                Ct = [(pxHat-P.LmX(j))/ct (pyHat-P.LmY(j))/ct 0 0 0 (P.LmX(j)-pxHat)/ct (P.LmY(j)-pyHat)/ct 0 0];
            
                else
                Ct = [(pxHat-P.LmX(3))/ct (pyHat-P.LmY(3))/ct 0 0 0 0 0 (P.LmX(3)-pxHat)/ct (P.LmY(3)-pyHat)/ct];
                
                end
            
            Lt = Pt*Ct'/(Rt+Ct*Pt*Ct');
            Pt = (eye(9)-Lt*Ct)*Pt;
            xHat = xHat + Lt*(rho10-ct);
        end
end
    tPr = t;
    out = [xHat;diag(Pt)];
end
