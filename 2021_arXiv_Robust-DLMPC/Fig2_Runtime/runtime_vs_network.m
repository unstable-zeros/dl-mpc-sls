%% Run simulation
Ns       = [10 50 100 200];
d        = 4;
T        = 5;
seed     = 2021;
tHorizon = 20;

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

data_rvn_nom = cell(length(Ns), 1);
data_rvn_rob = cell(length(Ns), 1);

% Nominal but with solver instead of explicit solution
data_rvn_nomSolver = cell(length(Ns), 1);

for i = 1:length(Ns)
    fprintf('Scanning network size %d of %d\n', i, length(Ns));
    N = Ns(i);

    [sys, w, x0] = setup_plant(N, tHorizon, seed);
    % Runtime is relevant so be careful with convergence criteria
    paramsNom = setup_ctrller_partial(sys, d, T);

    paramsNom.eps_p_    = 8e-3;
    paramsNom.eps_d_    = 8e-3;
    paramsNom.maxIters_ = 8000;
    paramsNom.rho_      = 1;
    paramsNom.tau_i_    = 1.5;
    paramsNom.tau_d_    = 1.5;
    paramsNom.muAdapt_  = 10;
    paramsNom.rhoMax_   = 3;   
    
    paramsNom.stateUB_      = 20 * ones(sys.Nx, 1);
    paramsNom.stateUB_(5)   = 1;
    paramsNom.stateLB_      = -paramsNom.stateUB_;    

    % Robust MPC has larger matrices, may require different conv params
    paramsRob = copy(paramsNom);

    data_rvn_nom{i} = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);
    data_rvn_rob{i} = run_scenarios(sys, paramsRob, tHorizon, w, x0, scenariosRob);

    paramsNom.solverMode_ = MPCSolverMode.UseSolver;
    data_rvn_nomSolver{i} = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);

end

% Sanity check: these should be relatively small
for i = 1:length(Ns)
    fprintf('N: %d\n', Ns(i));
    print_cent_dist_diff(data_rvn_nom{i}, 'Nominal');
    print_cent_dist_diff(data_rvn_rob{i}, 'Robust ');
end

save('data_rvn.mat');

%% Plot (not formatted for paper)
rvns = zeros(length(Ns), 3);
for i = 1:length(Ns)
    rvns(i, 1) = data_rvn_nom{i}{2}{3};       % nominal
    rvns(i, 2) = data_rvn_nomSolver{i}{2}{3}; % nominal using solver
    rvns(i, 3) = data_rvn_rob{i}{2}{3}; % robust
end

figure(1); hold on;
for i = 1:3
    plot(Ns, rvns(:, i));
end

xlabel('N');
ylabel('Runtime per state');

legend('Nominal (explicit)', 'Nominal', 'Robust');