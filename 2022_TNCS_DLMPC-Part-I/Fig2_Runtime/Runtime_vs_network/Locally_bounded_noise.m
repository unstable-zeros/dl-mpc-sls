%% Setup plant
gridSizes = [4 6 8 11]; % corresponding to network of 16, 36, 64, 121
seed          = 624;
connectThresh = 0.6; % lower = more connected grid

numSizes     = length(gridSizes);
plotTopology = false;
  
syss        = cell(numSizes, 1);
adjMtxs     = cell(numSizes, 1);
nodeCoordss = cell(numSizes, 1);
for i = 1:numSizes
    gridSize = gridSizes(i);
    [syss{i}, adjMtxs{i}, nodeCoordss{i}] = get_plant(gridSize, seed, connectThresh);

    if plotTopology
        figh = figure();
        plot_graph(adjMtxs{i}, nodeCoordss{i}, 'k');
        x = 10; y = 50; width = 580; height = 500;
        set(figh, 'Position', [x y width height]); % display only
    end
end

%% Run simulations
d        = 4;
T        = 5;
tHorizon = 20;
sigma    = 1;

scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

% Disturbances
rng(seed);

data_rvn_rob = cell(numSizes, 1); 

for i = 1:numSizes
    fprintf('Scanning network size %d of %d\n', i, numSizes);
    sys = syss{i};
    
    lbn = true; % get controller for locally bounded noise
    params = get_controller_params(sys, d, T, lbn);
    
    % Locally bound w (depends on d)
    x0 = rand(sys.Nx, 1);
    w  = 2 * rand(sys.Nx, tHorizon) - 1; % w in [-1, 1]
    w  = get_loc_bnd_noise(w, sys, params, sigma);
    
    params.eps_p_    = 8e-3;
    params.eps_d_    = 8e-3;
    params.maxIters_ = 8000;
    params.rho_      = 1;
    params.tau_i_    = 1.5;
    params.tau_d_    = 1.5;
    params.muAdapt_  = 10;
    params.rhoMax_   = 5;
    
    data_rvn_rob{i} = run_scenarios(sys, params, tHorizon, w, x0, scenariosRob);
end

% Sanity check: these should be relatively small
for i = 1:numSizes
    fprintf('numNodes: %d\n', gridSizes(i).^2);
    print_cent_dist_diff(data_rvn_rob{i}, 'Robust ');
end

save('data_rvn.mat');

%% Plot (not formatted for paper)
rvns = zeros(numSizes, 1);
for i = 1:numSizes
    rvns(i) = data_rvn_rob{i}{2}{3}; % robust    
end

figure(1); hold on;
plot(gridSizes.^2, rvns);

xlabel('N');
ylabel('Runtime per state, robust');