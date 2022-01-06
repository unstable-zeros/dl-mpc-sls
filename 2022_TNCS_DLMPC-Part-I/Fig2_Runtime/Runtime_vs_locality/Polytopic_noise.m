%% Setup plant
gridSize      = 4; % total size = gridSize^2
seed          = 621;
connectThresh = 0.65; % lower = more connected grid

plotTopology = false;

[sys, adjMtx, nodeCoords] = get_plant(gridSize, seed, connectThresh);

if plotTopology
    fig1h = figure(1);
    plot_graph(adjMtx, nodeCoords, 'k');

    x = 10; y = 50; width = 580; height = 500;
    set(fig1h, 'Position', [x y width height]); % display only
end

%% Run simulations
ds       = [4 5 7 10];
T        = 5;
tHorizon = 20;

scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

% Disturbances
rng(seed);
x0 = rand(sys.Nx, 1);
w  = 2 * rand(sys.Nx, tHorizon) - 1; % w in [-1, 1]

data_rvl_rob = cell(length(ds), 1); 

for i = 1:length(ds)
    fprintf('Scanning neighborhood size %d of %d\n', i, length(ds));
    d = ds(i);

    % Runtime is relevant so be careful with convergence criteria
    params = get_controller_params(sys, d, T);

    params.eps_p_    = 8e-3;
    params.eps_d_    = 8e-3;
    params.maxIters_ = 8000;
    params.rho_      = 1;
    params.tau_i_    = 1.5;
    params.tau_d_    = 1.5;
    params.muAdapt_  = 10;
    params.rhoMax_   = 5;
    
    data_rvl_rob{i} = run_scenarios(sys, params, tHorizon, w, x0, scenariosRob);
end

% Sanity check: these should be relatively small
for i = 1:length(ds)
    fprintf('d: %d\n', ds(i));
    print_cent_dist_diff(data_rvl_rob{i}, 'Robust ');
end

save('data_rvl.mat');

%% Plot (not formatted for paper)
rvls = zeros(length(ds), 1);
for i = 1:length(ds)
    rvls(i) = data_rvl_rob{i}{2}{3}; % robust    
end

figure(1); hold on;
plot(ds, rvls);

xlabel('d');
ylabel('Runtime per state, robust');