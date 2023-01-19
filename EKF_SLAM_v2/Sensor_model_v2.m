%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%
%%
function out = Sensor_model_v2(in,P)

%% IMU - odometry data

    NN   = 0;
    v    = in(NN+1) + P.spv*randn(1,1);
    w    = in(NN+2) + P.spw*randn(1,1);
%% True Vehicle States 

    NN   = NN+2;
    pxt  = in(NN+1);
    pyt  = in(NN+2);
    psit = in(NN+3);
   
%% time
    
    NN = NN + 3;
    t    = in(NN+1);
%% Range Sensor

    rho = zeros(P.NumLm,1);
    
    for i = 1:1:P.NumLm
        rho(i) = sqrt((pxt-P.LmX(i))^2+(pyt-P.LmY(i))^2) + P.spr1*randn(1,1);
    end
%% Bearing sensor 

    chi = zeros(P.NumLm, 1);
    
    for i = 1:1:P.NumLm
        chi(i) = (atan2((P.LmY(i) - pyt),(P.LmX(i)-pxt)) - psit + P.spr2*randn(1,1));
    end 
    
%% Magnetic compass 

    theta = zeros(P.NumLm, 1);
    
    for i = 1:1:P.NumLm
        theta(i) = wrapToPi(psit) + P.spr3*randn(1,1);
    end 
        
 
    
%% Output
    out = [...
        v;...
        w;...
        rho;...
        chi;...
        theta;...
        t;...
        ];
end  