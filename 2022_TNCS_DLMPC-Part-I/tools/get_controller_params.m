function params = get_controller_params(sys, d, T, loc_bnd_noise)
% Sets up the part of the controller that is common across examples
% d is local patch size
% T is time horizon of MPC controller
% loc_bnd_noise: specify as true to specify locally bounded noise
%                instead of polytopic noise
% Missing are eps_p_, eps_d_, maxIters, and optional adaptive ADMM params

params = MPCParams();
params.locality_ = d;
params.tFIR_     = T;

params.QSqrt_    = eye(sys.Nx);
params.RSqrt_    = eye(sys.Nu);

% Input and state constraints
params.stateConsMtx_ = eye(sys.Nx);
params.stateUB_      = 20 * ones(sys.Nx, 1); % Freq constraints, loose

polytopic_noise = true;
if nargin == 4
    if loc_bnd_noise
        polytopic_noise = false;
    end
end

% Disturbance constraints
if polytopic_noise
    params.distConsMtx_ = eye(sys.Nx);
    params.distUB_      = 1 * ones(sys.Nx, 1);
    params.distLB_      = -params.distUB_;
    
    % polynoise or noiseless
    params.stateUB_(1:2:sys.Nx) = 4; % Phase constraints
    params.stateLB_ = -params.stateUB_;
else % locally bounded noise
    params.locNoiseBound_ = 1;
    
    params.stateUB_(1:2:sys.Nx) = 3; % Phase constraints
    params.stateLB_             = -params.stateUB_;
end 
    
end