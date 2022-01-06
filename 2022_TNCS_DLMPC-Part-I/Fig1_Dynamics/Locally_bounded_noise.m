%% Visualize grid if needed
gridSize      = 3; % total size = gridSize^2
seed          = 619;
connectThresh = 0.65; % lower = more connected grid

plotNode = 4;
plotTopology = false;

[sys, adjMtx, nodeCoords] = get_plant(gridSize, seed, connectThresh);

if plotTopology
    fig1h = figure(1);
    plot_graph(adjMtx, nodeCoords, 'k');
    plot_special_vertex(plotNode, nodeCoords, 'r');

    x = 10; y = 50; width = 580; height = 500;
    set(fig1h, 'Position', [x y width height]); % display only
end

%% Setup + simulate
d        = 4; % Note: this "d" is actually d+1 (using paper definitions)
T        = 5;
tHorizon = 20;
sigma    = 1;

% Disturbances
rng(seed);

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

lbn = true; % get controller for locally bounded noise
paramsNom = get_controller_params(sys, d, T, lbn);

% Locally bound w (depends on d)
x0 = rand(sys.Nx, 1); x0(2:2:end) = 0;
w  = 2 * rand(sys.Nx, tHorizon) - 1; % w in [-1, 1]
w  = get_loc_bnd_noise(w, sys, paramsNom, sigma);

paramsNom.stateUB_(1:2:end) = 10;         % looser phase constraints
paramsNom.stateLB_ = -paramsNom.stateUB_; % symmetrical phase constraints

paramsNom.stateLB_(2:2:end) = -0.5; % tight constraints
paramsNom.stateUB_(2:2:end) = 1.5; % asymmetrical freq constraints

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

data_dynamics_nom = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);
data_dynamics_rob = run_scenarios(sys, paramsRob, tHorizon, w, x0, scenariosRob);

% Sanity check: these should be relatively small
print_cent_dist_diff(data_dynamics_nom, 'Nominal');
print_cent_dist_diff(data_dynamics_rob, 'Robust (LBN)');

save('dynamics_lbn.mat');

%% Plot (not formatted for paper)
plotNode = 4;

phaseIdx  = plotNode*2-1;
freqIdx   = plotNode*2;

time = 1:tHorizon;
freqLBs = paramsNom.stateLB_(freqIdx)*ones(length(time)+2, 1);

figure(2);
subplot(2,1,1); title('Phase'); hold on;
plot(time, data_dynamics_nom{1}{1}(phaseIdx,:));
plot(time, data_dynamics_rob{1}{1}(phaseIdx,:));
plot(time, data_dynamics_nom{2}{1}(phaseIdx,:), 'o');
plot(time, data_dynamics_rob{2}{1}(phaseIdx,:), '*');
subplot(2,1,2); title('Frequency'); hold on;
plot(time, data_dynamics_nom{1}{1}(freqIdx,:));
plot(time, data_dynamics_rob{1}{1}(freqIdx,:));
plot(time, data_dynamics_nom{2}{1}(freqIdx,:), 'o');
plot(time, data_dynamics_rob{2}{1}(freqIdx,:), '*');
plot([0 time 20], freqLBs, '--r', 'HandleVisibility', 'off');
xlabel('Time');

legend('CentNominal', 'CentRobust (LBN)', 'DistNominal', 'DistRobust (LBN)');