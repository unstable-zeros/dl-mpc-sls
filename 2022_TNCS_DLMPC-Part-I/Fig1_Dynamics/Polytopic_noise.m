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
w  = 2 * rand(sys.Nx, tHorizon) - 1; % w in [-1, 1]

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

paramsNom = get_controller_params(sys, d, T);
paramsNom.stateUB_(1:2:sys.Nx) = 3.2; % tighter phase constraints
paramsNom.stateLB_ = -paramsNom.stateUB_;

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
print_cent_dist_diff(data_dynamics_rob, 'Robust ');

save('dynamics_pn.mat');

%% Plot (not formatted for paper)
phaseIdx  = plotNode*2-1;
freqIdx   = plotNode*2;

time = 1:tHorizon;
phaseUBs = paramsNom.stateUB_(1)*ones(length(time)+2, 1);

figure(2);
subplot(2,1,1); title('Phase'); hold on;
plot(time, data_dynamics_nom{1}{1}(phaseIdx,:));
plot(time, data_dynamics_rob{1}{1}(phaseIdx,:));
plot(time, data_dynamics_nom{2}{1}(phaseIdx,:), 'o');
plot(time, data_dynamics_rob{2}{1}(phaseIdx,:), '*');
plot([0 time 20], phaseUBs, '--r', 'HandleVisibility', 'off');
subplot(2,1,2); title('Frequency'); hold on;
plot(time, data_dynamics_nom{1}{1}(freqIdx,:));
plot(time, data_dynamics_rob{1}{1}(freqIdx,:));
plot(time, data_dynamics_nom{2}{1}(freqIdx,:), 'o');
plot(time, data_dynamics_rob{2}{1}(freqIdx,:), '*');
xlabel('Time');

legend('CentNominal', 'CentRobust', 'DistNominal', 'DistRobust');