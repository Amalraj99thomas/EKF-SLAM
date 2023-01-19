%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%EKF for SLAM - with range, bearing and magnetometer
%Author: Anuraga Sankepally 
%Created on: 8-12-2021
%Edited on: 
%Version: 2

function out = Estimator_SLAM4(in,P)

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
    
    ft = zeros(5,1);
    
%%  Initialize 
    persistent xHat Pt tPr
    
    if t == 0 
        
        xHat = [0;0;0;1.2;2.4]+0.1*randn(5,1);
        Pt = 0.1*eye(5);
        tPr = t;
    
    else 
            Ts = t - tPr;
            steps = 10;
        for i = 1:1:steps
            
            NN = 0;
            pxHat = xHat(NN+1);
            pyHat = xHat(NN+2);
            psiHat = xHat(NN+3);
            
            NN = NN + 3;
            
            LmxHat = xHat(NN+1);
            LmyHat = xHat(NN+2);
            
            
            pxHat_dot = v*cos(psiHat);
            pyHat_dot = v*sin(psiHat);
            psiHat_dot = w;
            
            %ft = zeros(1,3+2*P.NumLm);
            ft(1,1) = pxHat_dot;
            ft(2,1) = pyHat_dot;
            ft(3,1) =  psiHat_dot;
            
            xHat = xHat + (Ts/steps)*ft;
            

            At = [...
                    0 0 -v*sin(psiHat) 0 0 0 0 0 0; ...
                    0 0  v*cos(psiHat) 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    0 0              0 0 0 0 0 0 0;...
                    ];

            B = [...
                    cos(psiHat) 0;...
                    sin(psiHat) 0;...
                              0 1;...
                              0 0;...
                              0 0;...
                  ];
                  
            Qu = B*qu*B';
            Pt = Pt + (Ts/steps)*(Qu + At*Pt + Pt*At');
        end
%% Measurement Updates
           
%% range sensor            
  if P.range_flag == 1 
      
        NN = 0;

        pxHat = xHat(NN+1);
        pyHat = xHat(NN+2);
        psiHat = wrapToPi(xHat(NN+3));
        
        NN = NN + 3; 
        LmXhat = xHat(NN+1);
        LmYhat = xHat(NN+2);

        ct = sqrt((pxHat-LmXhat)^2+(pyHat-LmYhat)^2);

        rho10 = rho;%(j);

        Ct = [(pxHat-LmXhat)/ct (pyHat-LmYhat)/ct 0 (LmXhat-pxHat)/ct (LmYhat-pyHat)/ct];

        Lt = Pt*Ct'/(Rt1+Ct*Pt*Ct');
        Pt = (eye(5)-Lt*Ct)*Pt;
        xHat = xHat + Lt*(rho10-ct);

  end 
 %% Bearing sensor 
% if P.bearing_flag == 1
%      
%      for j = 1:1:P.NumLm
%                 
%         NN = 0;
% 
%         pxHat = xHat(NN+1);
%         pyHat = xHat(NN+2);
%         psiHat = wrapToPi(xHat(NN+3));
%         xHat(NN+3) = wrapToPi(xHat(NN+3));
%         mHat   = xHat(NN+4:NN+9);
% 
%         bt = atan2((pyHat-P.LmY(j)),(pxHat-P.LmX(j)))+ pi/2 - psiHat;
%         chi10   = chi(j);
% 
%         lx1 = P.LmX(1);
%         lx2 = P.LmX(2);
%         lx3 = P.LmX(3);
% 
%         ly1 = P.LmY(1);
%         ly2 = P.LmY(2);
%         ly3 = P.LmY(3);
% 
%         if j == 1 
% 
%             Bt = [(ly1 - pyHat)/((ly1 - pyHat)^2 + (lx1 - pxHat)^2), -(lx1 - pxHat)/((ly1 - pyHat)^2 + (lx1 - pxHat)^2), -1, -(ly1 - pyHat)/((ly1 - pyHat)^2 + (lx1 - pxHat)^2), (lx1 - pxHat)/((ly1 - pyHat)^2 + (lx1 - pxHat)^2), 0, 0, 0, 0];
% 
%         elseif j == 2 
%             Bt = [(ly2 - pyHat)/((ly2 - pyHat)^2 + (lx2 - pxHat)^2), -(lx2 - pxHat)/((ly2 - pyHat)^2 + (lx2 - pxHat)^2), -1, 0, 0, -(ly2 - pyHat)/((ly2 - pyHat)^2 + (lx2 - pxHat)^2), (lx2 - pxHat)/((ly2 - pyHat)^2 + (lx2 - pxHat)^2), 0, 0];
% 
%         else
%             Bt = [(ly3 - pyHat)/((ly3 - pyHat)^2 + (lx3 - pxHat)^2), -(lx3 - pxHat)/((ly3 - pyHat)^2 + (lx3 - pxHat)^2), -1, 0, 0, 0, 0, -(ly3 - pyHat)/((ly3 - pyHat)^2 + (lx3 - pxHat)^2), (lx3 - pxHat)/((ly3 - pyHat)^2 + (lx3 - pxHat)^2)];
% 
%         end
% 
%         Lt = Pt*Bt'/(Rt2+Bt*Pt*Bt');
%         Pt = (eye(9)-Lt*Bt)*Pt;
%         xHat = xHat + Lt*(wrapToPi(chi10)-wrapToPi(bt));
%      end 
% end 
 %% Magnetic compass 
 
if P.magneto_flag == 1
    
%     for j = 1:1:P.NumLm

        NN = 0;

        pxHat = xHat(NN+1);
        pyHat = xHat(NN+2);
        psiHat = wrapToPi(xHat(NN+3));
        
                
        NN = NN + 3; %+ 2*(j-1);
        
        LmXhat = xHat(NN+1);
        LmYhat = xHat(NN+2);
               

        mt = psiHat;
        mag10 = theta;

        Mt = [0 0 1 0 0 ];

        Lt = Pt*Mt'/(Rt3+Mt*Pt*Mt');
        Pt = (eye(3+2*P.NumLm)-Lt*Mt)*Pt;
        xHat = xHat + Lt*(mag10-mt);
        
%     end 
end 
   
            

end
    tPr = t;
    out = [xHat;diag(Pt)];
end
