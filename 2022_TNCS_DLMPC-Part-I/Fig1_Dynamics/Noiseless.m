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

% Disturbances
rng(seed);
x0 = rand(sys.Nx, 1);
w  = zeros(sys.Nx, tHorizon);

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};

params = get_controller_params(sys, d, T);

params.stateLB_(2:2:end) = -0.3;
params.stateUB_ = -params.stateLB_;

params.eps_p_    = 2e-3;
params.eps_d_    = 2e-3;
params.maxIters_ = 8000;
params.rho_      = 1;
params.tau_i_    = 1.5;
params.tau_d_    = 1.5;
params.muAdapt_  = 10;
params.rhoMax_   = 5;

data_dynamics_nom = run_scenarios(sys, params, tHorizon, w, x0, scenariosNom);

% Sanity check: these should be relatively small
print_cent_dist_diff(data_dynamics_nom, 'Nominal');

save('dynamics_noiseless.mat');

%% Plot (not formatted for paper)
phaseIdx  = plotNode*2-1;
freqIdx   = plotNode*2;

time = 1:tHorizon;
freqLBs = params.stateLB_(freqIdx)*ones(length(time)+2, 1);

figure(2);
subplot(2,1,1); title('Phase'); hold on;
plot(time, data_dynamics_nom{1}{1}(phaseIdx,:));
plot(time, data_dynamics_nom{2}{1}(phaseIdx,:), 'o');
subplot(2,1,2); title('Frequency'); hold on;
plot(time, data_dynamics_nom{1}{1}(freqIdx,:));
plot(time, data_dynamics_nom{2}{1}(freqIdx,:), 'o');
plot([0 time 20], freqLBs, '--r', 'HandleVisibility', 'off');
xlabel('Time');

legend('CentNominal', 'DistNominal');