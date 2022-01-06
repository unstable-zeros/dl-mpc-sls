%% Run simulation where nominal coincides with robust
% i.e. no expected disturbance, no actual disturbance
N        = 10;
d        = 4;
T        = 5;
seed     = 2021;
tHorizon = 20;

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

[sys, w, x0] = setup_plant(N, tHorizon, seed);

% No disturbance
w = zeros(sys.Nx, tHorizon);

paramsNom = setup_ctrller_partial(sys, d, T);

ub = 20 * ones(sys.Nx, 1);
ub([1,3,5,6,8,10]) = 1.5;
paramsNom.stateUB_  = ub;
paramsNom.stateLB_  = -ub;
    
paramsNom.eps_p_    = 2e-3;
paramsNom.eps_d_    = 2e-3;
paramsNom.maxIters_ = 8000;
paramsNom.rho_      = 1;
paramsNom.tau_i_    = 1.5;
paramsNom.tau_d_    = 1.5;
paramsNom.muAdapt_  = 10;
paramsNom.rhoMax_   = 5;

% Robust MPC has larger matrices, may require different conv params
paramsRob = copy(paramsNom);
paramsRob.distUB_ = zeros(sys.Nx, 1); % No expected disturbance
paramsRob.distLB_ = zeros(sys.Nx, 1);

% To match RNG from cost_vs_locality script
x0 = rand(sys.Nx, 1);
x0 = rand(sys.Nx, 1);
x0 = rand(sys.Nx, 1);

data_dynamics_nom = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);
data_dynamics_rob = run_scenarios(sys, paramsRob, tHorizon, w, x0, scenariosRob);

% Sanity check: these should be relatively small
print_cent_dist_diff(data_dynamics_nom, 'Nominal');
print_cent_dist_diff(data_dynamics_rob, 'Robust ');

save('data_dynamics_noiseless.mat');

%% Plot (not formatted for paper)
plotState = 3;

figure(1); hold on;
time = 1:tHorizon;
plot(time, data_dynamics_nom{1}{1}(plotState,:));
plot(time, data_dynamics_rob{1}{1}(plotState,:));
plot(time, data_dynamics_nom{2}{1}(plotState,:), 'o');
plot(time, data_dynamics_rob{2}{1}(plotState,:), '*');

xlabel('Time');
ylabel('State');

legend('CentNominal', 'CentRobust', 'DistNominal', 'DistRobust');