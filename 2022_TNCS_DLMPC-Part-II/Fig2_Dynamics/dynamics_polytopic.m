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
tHorizon = 20;

% Disturbances
rng(seed);
x0          = zeros(sys.Nx, 1);
x0(1:2:end) = 3; % unactuated
x0(2:2:end) = -20; % actuated
w           = 2 * rand(sys.Nx, tHorizon) - 1; % w in [-1, 1]

scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

params = MPCParams();
params.locality_ = 4;
params.tFIR_     = 2;

params.QSqrt_    = eye(sys.Nx);
params.RSqrt_    = eye(sys.Nu);

params.stateConsMtx_        = eye(sys.Nx);
params.stateUB_             = 20 * ones(sys.Nx, 1); % Freq
params.stateUB_(1:2:sys.Nx) = 3; % Phase
params.stateLB_             = -params.stateUB_;

params.distConsMtx_ = eye(sys.Nx);
params.distUB_      =  1 * ones(sys.Nx, 1);
params.distLB_      = -1 * ones(sys.Nx, 1);

params.eps_p_    = 8e-3;
params.eps_d_    = 8e-3;
params.maxIters_ = 10000;
params.rho_      = 1;
params.tau_i_    = 1.5;
params.tau_d_    = 1.5;
params.muAdapt_  = 10;
params.rhoMax_   = 5;

% Consensus parameters for terminal cost
params.mu_           = 1.5;
params.eps_x_        = 2e-3;
params.eps_z_        = 2e-3;
params.maxItersCons_ = 500;

%% Synthesize w/o terminal set
fprintf('Doing MPC with *no* terminal set...\n')
params.mode_        = MPCMode.Centralized;
[xCentB, uCentB, ~] = sls_mpc(sys, x0, w, params, tHorizon);

params.mode_        = MPCMode.Distributed;
[xB, uB, statsB]    = sls_mpc(sys, x0, w, params, tHorizon);

fprintf('MPC w/terminal set: avgTime: %.4f, avgIters: %.4f\n\n', statsB.time_, statsB.iters_);

objTermDist = get_cost_fn(params, xB, uB);
objTermCent = get_cost_fn(params, xCentB, uCentB);
fprintf('Distributed cost w/terminal set: %f\n', objTermDist);
fprintf('Centralized cost w/terminal set: %f\n', objTermCent);

name = strcat('dyn_noguar_polytopic_seed',num2str(seed),'_ct',num2str(ct*100),'.mat');
save(name);

%% Add terminal set + synthesize w/ terminal set
fprintf('Synthesizing robust terminal set...\n')
[params, tStats] = terminal_set(sys, params);
params = remove_redundancy_terminal(sys, params);
fprintf('Robust erminal set: avgTime: %.4f, avgIters: %.4f\n\n', tStats.time_, tStats.iters_);

fprintf('Doing rMPC *with* terminal set...\n')
params.mode_        = MPCMode.Centralized;
[xCentB, uCentB, ~] = sls_mpc(sys, x0, w, params, tHorizon);

params.mode_        = MPCMode.Distributed;
[xB, uB, statsB]    = sls_mpc(sys, x0, w, params, tHorizon);

fprintf('rMPC w/terminal set: avgTime: %.4f, avgIters: %.4f\n\n', statsB.time_, statsB.iters_);

objTermDist = get_cost_fn(params, xB, uB);
objTermCent = get_cost_fn(params, xCentB, uCentB);
fprintf('Distributed cost w/terminal set: %f\n', objTermDist);
fprintf('Centralized cost w/terminal set: %f\n', objTermCent);

name = strcat('dyn_tset_polytopic_seed',num2str(seed),'_ct',num2str(ct*100),'.mat');
save(name);

%% Add terminal cost & set + synthesize w/ terminal cost & set

params.terminal_cost_ = true;

fprintf('Doing MPC with terminal constraint + cost...\n')
params.mode_        = MPCMode.Centralized;
[xCentC, uCentC, ~] = sls_mpc(sys, x0, w, params, tHorizon);

params.mode_        = MPCMode.Distributed;
[xC, uC, statsC]    = sls_mpc(sys, x0, w, params, tHorizon);

fprintf('MPC, terminal constraint+cost:\n');
fprintf('avgTime: %.4f, avgIters: %.4f, avgConsIters: %.4f\n\n', statsC.time_, statsC.iters_, statsC.consIters_);

objDistC = get_cost_fn(params, xC, uC);
objCentC = get_cost_fn(params, xCentC, uCentC);
fprintf('Dist. cost w/terminal constraint + cost: %f\n', objDistC);
fprintf('Cent. cost w/terminal constraint + cost: %f\n', objCentC);

name = strcat('dyn_tcost_polytopic_seed',num2str(seed),'_ct',num2str(ct*100),'.mat');
save(name);

%% Plot (not formatted for paper)
phaseIdx  = plotNode*2-1;
freqIdx   = plotNode*2;

time = 1:tHorizon;
phaseLBs = params.stateLB_(phaseIdx)*ones(length(time), 1);

figure(2);
subplot(2,1,1); title('Phase'); hold on;
plot(time, xCentB(phaseIdx, :), 'b');
plot(time, xB(phaseIdx, :), '*b');
plot(time, xCentA(phaseIdx, :), 'g');
plot(time, phaseLBs, '--r', 'HandleVisibility', 'off');
legend('Centralized [Term]', 'Distributed [Term]', 'Centralized [NoTerm]')

subplot(2,1,2); title('Frequency'); hold on;
plot(time, xCentB(freqIdx, :), 'b');
plot(time, xB(freqIdx, :), '*b');
plot(time, xCentA(freqIdx, :), 'g');
xlabel('Time');
