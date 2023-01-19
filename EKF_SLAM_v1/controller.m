function out= controller(in,P)
    NN = 0;
    x     = in(NN+1);
    y     = in(NN+2);
    psi   = in(NN+3);
    NN = NN + 3;
    m     = in(NN+1:NN+2*P.NumLm);
    NN = NN + 2*P.NumLm;
    t     = in(NN +1);
    r     = P.radius;
    V     = 0.5;
    omega = V/r;
    out   = [V;omega];
end