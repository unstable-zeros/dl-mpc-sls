function params = setup_ctrller_partial(sys, d, T)
% Sets up the part of the controller that is common across examples
% d is local patch size
% T is time horizon of MPC controller
% Missing are eps_p_, eps_d_, maxIters, and optional adaptive ADMM params

params = MPCParams();
params.locality_ = d;
params.tFIR_     = T;

params.QSqrt_    = eye(sys.Nx);
params.RSqrt_    = eye(sys.Nu);

% Input and state constraints
params.stateConsMtx_ = eye(sys.Nx);
params.stateUB_      = 10 * ones(sys.Nx, 1);
params.stateUB_(5)   = 1;
params.stateLB_      = -params.stateUB_;    

% Disturbance constraints
params.distConsMtx_ = eye(sys.Nx);
params.distUB_      = 1 * ones(sys.Nx, 1);  
params.distLB_      = -params.distUB_;

end