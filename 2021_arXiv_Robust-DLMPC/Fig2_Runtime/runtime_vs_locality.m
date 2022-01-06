%% Run simulation
N        = 15;
ds       = [4 5 7 10];
T        = 5;
seed     = 2020;
tHorizon = 20;

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

[sys, w, x0] = setup_plant(N, tHorizon, seed);

data_rvl_nom = cell(length(ds), 1); 
data_rvl_rob = cell(length(ds), 1); 

% Nominal but with solver instead of explicit solution
data_rvl_nomSolver = cell(length(ds), 1);

for i = 1:length(ds)
    fprintf('Scanning neighborhood size %d of %d\n', i, length(ds));
    d = ds(i);

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
    
    paramsNom.solverMode_ = MPCSolverMode.UseSolver;

    % Robust MPC has larger matrices, may require different conv params
    paramsRob = copy(paramsNom);

    data_rvl_nom{i} = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);
    data_rvl_rob{i} = run_scenarios(sys, paramsRob, tHorizon, w, x0, scenariosRob);

    paramsNom.solverMode_ = MPCSolverMode.UseSolver;
    data_rvl_nomSolver{i} = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);
end

% Sanity check: these should be relatively small
for i = 1:length(ds)
    fprintf('d: %d\n', ds(i));
    print_cent_dist_diff(data_rvl_nom{i}, 'Nominal');
    %print_cent_dist_diff(data_rvl_rob{i}, 'Robust ');
end

save('data_rvl.mat');

%% Plot (not formatted for paper)
rvls = zeros(length(ds), 3);
for i = 1:length(ds)
    rvls(i, 1) = data_rvl_nom{i}{2}{3};       % nominal
    rvls(i, 2) = data_rvl_nomSolver{i}{2}{3}; % nominal using solver
    rvls(i, 3) = data_rvl_rob{i}{2}{3}; % robust    
end

figure(1); hold on;
for i = 1:3
    plot(ds, rvls(:, i));
end

xlabel('d');
ylabel('Runtime per state');

legend('Nominal (explicit)', 'Nominal', 'Robust');