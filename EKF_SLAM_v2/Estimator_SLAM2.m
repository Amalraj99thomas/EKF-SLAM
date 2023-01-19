%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%EKF for SLAM - with range, bearing and magnetometer
%Author: Anuraga Sankepally 
%Created on: 8-12-2021
%Edited on: 
%Version: 2

function out = Estimator_SLAM2(in,P)

%% IMU inputs
    NN = 0;
    v = in(NN+1);
    w = in(NN+2);
%% Range Measurements

    NN = NN+2;
    rho = in(NN+1:NN+P.NumLm);
    
%% Bearing Measurements 

    NN = NN+P.NumLm;
    chi = in(NN+1:NN+P.NumLm);
    
%% Compass Measurements 

    NN = NN+P.NumLm;
    theta = in(NN+1:NN+P.NumLm);
 
%% Time
    NN = NN+P.NumLm;
    t = in(NN+1);
%%
    qu = zeros(2,2);
    qu(1,1) = P.spv^2;
    qu(2,2) = P.spw^2;
%%  
    Rt1 = P.spr1^2;
    Rt2 = P.spr2^2;
    Rt3 = P.spr3^2;
    
    ft = zeros(3+2*P.NumLm,1);
    
%%  Initialize 
    persistent xHat Pt tPr
    
    if t == 0 
        
        xHat = [0;0;0;1.2;2.4;3.5;5.1;5.2;6.3]+[0.1*randn(3,1);0.01*randn(6,1)];
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
            
            %ft = zeros(1,3+2*P.NumLm);
            ft(1,1) = pxHat_dot;
            ft(2,1) = pyHat_dot;
            ft(3,1) =  psiHat_dot;
            
            xHat = xHat + (Ts/steps)*ft;
            
%             At = zeros((3+2*P.NumLm),(3+2*P.NumLm));
%             
%             At(1,3)= -v*sin(psiHat);
%             At(2,3)= v*cos(psiHat);
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

%             B = zeros((3+2*P.NumLm),2);
%             B(1,1) = cos(psiHat);
%             B(2,1) = sin(psiHat);
%             B(3,2) = 1;
            B = [...
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
                  
            Qu = B*qu*B';
            Pt = Pt + (Ts/steps)*(Qu + At*Pt + Pt*At');
        end
%% Measurement Updates
           
%% range sensor            
        if P.range_flag == 1 
      
            for j = 1:1:P.NumLm      
                NN = 0;

                pxHat = xHat(NN+1);
                pyHat = xHat(NN+2);
                psiHat = wrapToPi(xHat(NN+3));
        
                NN = NN + 3 + 2*(j-1);
                LmXhat = xHat(NN+1);
                LmYhat = xHat(NN+2);
                %mHat   = xHat(NN+4:NN+9);

                ct = sqrt((pxHat-LmXhat)^2+(pyHat-LmYhat)^2);

                rho10 = rho(j);

                if j == 1 

                    Ct = [(pxHat-LmXhat)/ct (pyHat-LmYhat)/ct 0 (LmXhat-pxHat)/ct (LmYhat-pyHat)/ct 0 0 0 0];

                else
                    if j == 2
                        Ct = [(pxHat-LmXhat)/ct (pyHat-LmYhat)/ct 0 0 0 (LmXhat-pxHat)/ct (LmYhat-pyHat)/ct 0 0];
                    else
                        Ct = [(pxHat-LmXhat)/ct (pyHat-LmYhat)/ct 0 0 0 0 0 (LmXhat-pxHat)/ct (LmYhat-pyHat)/ct];
                    end
                end
                Lt = Pt*Ct'/(Rt1+Ct*Pt*Ct');
                Pt = (eye(9)-Lt*Ct)*Pt;
                xHat = xHat + Lt*(rho10-ct);
            end 
        end 
 %% Bearing sensor 
        if P.bearing_flag == 1
     
            for j = 1:1:P.NumLm
                
                NN = 0;

                pxHat = xHat(NN+1);
                pyHat = xHat(NN+2);
                psiHat = xHat(NN+3);
                %xHat(NN+3) = wrapToPi(xHat(NN+3));
                NN = NN + 3 + 2*(j-1);
                
                LmXhat = xHat(NN+1);
                LmYhat = xHat(NN+2);

                bt = atan2((LmYhat-pyHat),(LmXhat-pxHat)) - psiHat;
                chi10   = chi(j);

                if j == 1 

                    Bt = [(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -(LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -1, -(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), (LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), 0, 0, 0, 0];

                else
                    if j == 2
                        Bt = [(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -(LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -1, 0, 0, -(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), (LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), 0, 0];
                    else
                        Bt = [(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -(LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -1, 0, 0, 0, 0, -(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), (LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2)];
                    end
                end

                Lt = Pt*Bt'/(Rt2+Bt*Pt*Bt');
                Pt = (eye(9)-Lt*Bt)*Pt;
                xHat = xHat + Lt*wrapToPi((chi10)-(bt));
            end 
        end 
 %% Magnetic compass 
 
        if P.magneto_flag == 1
    
            for j = 1:1:P.NumLm

                NN = 0;

                pxHat = xHat(NN+1);
                pyHat = xHat(NN+2);
                psiHat = wrapToPi(xHat(NN+3));
                
                NN = NN + 3 + 2*(j-1);
                
                LmXhat = xHat(NN+1);
                LmYhat = xHat(NN+2);
                mt = psiHat;
                mag10 = theta(j);

                Mt = [0 0 1 0 0 0 0 0 0];

                Lt = Pt*Mt'/(Rt3+Mt*Pt*Mt');
                Pt = (eye(9)-Lt*Mt)*Pt;
                xHat = xHat + Lt*(mag10-mt);
        
            end 
        end
    end
    tPr = t;
    out = [xHat;diag(Pt)];
end
