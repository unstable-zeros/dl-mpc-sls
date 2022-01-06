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

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
    
% Disturbances
rng(seed);
data_rvn_nom_offline  = cell(numSizes, 1); 
data_rvn_nom_withTerm = cell(numSizes, 1); 

for i = 1:numSizes
    fprintf('Scanning network size %d of %d\n', i, numSizes);
    sys = syss{i};
    
    x0 = rand(sys.Nx, 1) * 4 - 2; % in [-2, 2] 
    w  = zeros(sys.Nx, tHorizon); % noiseless
    
    % Runtime is relevant so be careful with convergence criteria
    params = get_controller_params(sys, d, T);
    
    % Noiseless
    params.distConsMtx_ = [];
    params.distUB_      = [];
    params.distLB_      = [];

    params.eps_p_    = 1e-3;
    params.eps_d_    = 1e-3;
    params.maxIters_ = 8000;
    params.rho_      = 1;
    params.tau_i_    = 1.5;
    params.tau_d_    = 1.5;
    params.muAdapt_  = 10;
    params.rhoMax_   = 5;
    
    %% Terminal set
    % Compute terminal set offline
    [params, tStats]        = terminal_set(sys, params);
    data_rvn_nom_offline{i} = [tStats.time_, tStats.iters_];
    params.terminal_cost_   = true;
    
    % Consensus parameters for terminal cost
    params.mu_           = 1.5;
    params.eps_x_        = 2e-3;
    params.eps_z_        = 2e-3;
    params.maxItersCons_ = 500;

    % Solve MPC online
    data_rvn_nom_withTerm{i} = run_scenarios(sys, params, tHorizon, w, x0, scenariosNom);    
end

% Sanity check: these should be relatively small
for i = 1:numSizes
    fprintf('numNodes: %d\n', gridSizes(i).^2);
    print_cent_dist_diff(data_rvn_nom_withTerm{i}, 'Nominal');
end

name = strcat('rvn_nom_term_seed',num2str(seed),'_ct',num2str(connectThresh*100),'.mat');
save(name);

%% Plot (not formatted for paper)
rvnsOnline  = zeros(numSizes, 1);
rvnsOffline = zeros(numSizes, 1); 
for i = 1:numSizes
    rvnsOnline(i)  = data_rvn_nom_withTerm{i}{2}{3};    
    rvnsOffline(i) = data_rvn_nom_offline{i}(1);
end

figure(1); 
subplot(2,1,1); hold on;
plot(gridSizes.^2, rvnsOnline);
title('Online runtime per state, noiseless');

subplot(2,1,2); hold on;
plot(gridSizes.^2, rvnsOffline);
title('Offline runtime per state, noiseless');

xlabel('N');
