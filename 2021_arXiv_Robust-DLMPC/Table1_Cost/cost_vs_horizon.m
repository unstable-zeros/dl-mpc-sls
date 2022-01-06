%% Run simulation
N        = 10;
d        = 4;
Ts       = [5 6 8 10];
seed     = 2021;
tHorizon = 20;

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

[sys, w, x0] = setup_plant(N, tHorizon, seed);

data_cvh_nom = cell(length(Ts), 1);
data_cvh_rob = cell(length(Ts), 1);

for i = 1:length(Ts)
    fprintf('Scanning time horizon size %d of %d\n', i, length(Ts));
    T = Ts(i);
    
    % For this one, since runtime not relevant, can use slightly different
    % convergence criteria for different cases if needed
    paramsNom = setup_ctrller_partial(sys, d, T);
    
    paramsNom.eps_p_    = 2e-3;
    paramsNom.eps_d_    = 2e-3;
    paramsNom.maxIters_ = 8000;
    paramsNom.rho_      = 1;
    paramsNom.tau_i_    = 1.5;
    paramsNom.tau_d_    = 1.5;
    paramsNom.muAdapt_  = 10;
    paramsNom.rhoMax_   = 2; 
    
    % Otherwise robust MPC can become infeasible for big horizons
    paramsNom.stateUB_      = 50 * ones(sys.Nx, 1);
    paramsNom.stateUB_(5)   = 1;
    paramsNom.stateLB_      = -paramsNom.stateUB_;    
    
    % Robust MPC has larger matrices, may require different conv params
    paramsRob = copy(paramsNom);
    % Adaptive ADMM
    paramsRob.eps_p_    = 8e-3;
    paramsRob.eps_d_    = 8e-3;
    
    data_cvh_nom{i} = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);
    data_cvh_rob{i} = run_scenarios(sys, paramsRob, tHorizon, w, x0, scenariosRob);
end

% Sanity check: these should be relatively small
for i = 1:length(Ts)
    fprintf('Ts: %d\n', Ts(i));
    print_cent_dist_diff(data_cvh_nom{i}, 'Nominal');
    print_cent_dist_diff(data_cvh_rob{i}, 'Robust ');
end

save('data_cvh.mat');

%% Output / plot (not formatted for paper)

cvhs = zeros(length(Ts), 4);
for i = 1:length(Ts)
    cvhs(i, 1) = data_cvh_nom{i}{1}{4}; % cent nominal
    cvhs(i, 2) = data_cvh_rob{i}{1}{4}; % cent robust
    cvhs(i, 3) = data_cvh_nom{i}{2}{4}; % dist nominal
    cvhs(i, 4) = data_cvh_rob{i}{2}{4}; % dist robust
end

marker = {'none','none','o', '*'};
    
figure(1); hold on;
for i = 1:4    
    plot(Ts, cvhs(:, i), 'Marker', marker{i});
end

xlabel('T');
ylabel('LQR Cost');
ylim([860 950])

legend('CentNominal', 'CentRobust', 'DistNominal', 'DistRobust');

% Print data as 'table'
Ts
cost_vs_horizon_nominal = cvhs(:, 3)'
cost_vs_horizon_robust  = cvhs(:, 4)'