

%%%%%%%%%% Local ekf for LN 3 %%%%%%%%%%
function out = Estimator_SLAML3(in,P)

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
    
%%  prediction
    
%%  Initialize 
persistent xHat Pt tPr
    
if t == 0 
        
        xHat = [0;0;0;5;8]+0.01*randn(5,1);
        Pt = 0.01*eye(5);
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
            
           
            ft(1,1) = pxHat_dot;
            ft(2,1) = pyHat_dot;
            ft(3,1) =  psiHat_dot;
            
            xHat = xHat + (Ts/steps)*ft;
            

            At = [...
                    0 0 -v*sin(psiHat) 0 0 ; ...
                    0 0  v*cos(psiHat) 0 0;...
                    0 0              0 0 0 ;...
                    0 0              0 0 0 ;...
                    0 0              0 0 0 ;...
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
 %% measurement update 
 
if P.range_flag == 1 
      
        NN = 0;

        pxHat = xHat(NN+1);
        pyHat = xHat(NN+2);
        psiHat = wrapToPi(xHat(NN+3));
        
        NN = NN + 3; 
        LmXhat = xHat(NN+1);
        LmYhat = xHat(NN+2);

        ct = sqrt((pxHat-LmXhat)^2+(pyHat-LmYhat)^2);

        rho10 = rho(1);%(j);

        Ct = [(pxHat-LmXhat)/ct (pyHat-LmYhat)/ct 0 (LmXhat-pxHat)/ct (LmYhat-pyHat)/ct];

        Lt = Pt*Ct'/(Rt1+Ct*Pt*Ct');
        Pt = (eye(5)-Lt*Ct)*Pt;
        xHat = xHat + Lt*(rho10-ct);

end

%% Bearing update
if P.bearing_flag == 1
                
        NN = 0;

        pxHat = xHat(NN+1);
        pyHat = xHat(NN+2);
        psiHat = xHat(NN+3);

        NN = NN + 3 ;

        LmXhat = xHat(NN+1);
        LmYhat = xHat(NN+2);

        bt = atan2((LmYhat-pyHat),(LmXhat-pxHat)) - psiHat;
        chi10   = chi(1);

        Bt = [(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -(LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), -1, -(LmYhat - pyHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2), (LmXhat - pxHat)/((LmYhat - pyHat)^2 + (LmXhat - pxHat)^2)];

        Lt = Pt*Bt'/(Rt2+Bt*Pt*Bt');
        Pt = (eye(5)-Lt*Bt)*Pt;
        xHat = xHat + Lt*wrapToPi((chi10)-(bt));
  
end 
%% Magnetometer update

if P.magneto_flag == 1

        NN = 0;

        pxHat = xHat(NN+1);
        pyHat = xHat(NN+2);
        psiHat = wrapToPi(xHat(NN+3));
        
        NN = NN + 3;
        LmXhat = xHat(NN+1);
        LmYhat = xHat(NN+2);

        mt = psiHat;
        mag10 = theta(1);

        Mt = [0 0 1 0 0];

        Lt = Pt*Mt'/(Rt3+Mt*Pt*Mt');
        Pt = (eye(9)-Lt*Mt)*Pt;
        xHat = xHat + Lt*(mag10-mt);

end 
end 
    
   tPr = t;
   out = [xHat;diag(Pt)];
   
end