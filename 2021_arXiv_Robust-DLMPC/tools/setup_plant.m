function [sys, w, x0] = setup_plant(N, tHorizon, seed)
% N is the number of subsystems
% d is local patch size
% T is time horizon of MPC controller
% seed is random number seed (for reproducibility of results)

rng(seed);

sys    = LTISystem;
sys.Nx = N; alpha = 0.8; rho = 2; actDens = 0.6; 
generate_dbl_stoch_chain(sys, rho, actDens, alpha);
sys.B1 = eye(sys.Nx);
sys.sanity_check_mpc();
  
x0   = rand(sys.Nx, 1);

% rand is uniform on [0,1], stretch to [-1, 1]
w  = 2 * rand(sys.Nx, tHorizon) - 1;

