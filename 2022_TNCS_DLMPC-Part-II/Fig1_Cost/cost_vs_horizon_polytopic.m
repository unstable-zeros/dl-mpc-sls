function cost_vs_horizon_polytopic(seed,connectThresh)

%% Visualize grid if needed
gridSize      = 3; % total size = gridSize^2
%seed          = 619;~
%connectThresh = 0.65; % lower = more connected grid

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
Ts       = [4 5 8 10];

scenariosRob = {ScenarioType.CentRobust};%, ScenarioType.DistRobust};

% Disturbances
rng(seed);
x0 = rand(sys.Nx, 1) * 4 - 2; % in [-2, 2] 
w  = 2 * rand(sys.Nx, tHorizon) - 1; % w in [-1, 1]

params = MPCParams();
params.locality_ = 4;

params.QSqrt_    = eye(sys.Nx);
params.RSqrt_    = eye(sys.Nu);

params.stateConsMtx_        = eye(sys.Nx);
params.stateUB_             = 20 * ones(sys.Nx, 1); % Freq
params.stateUB_(1:2:sys.Nx) = 3; % Phase
params.stateLB_             = -params.stateUB_;

params.distConsMtx_ = eye(sys.Nx);
params.distUB_      =  1 * ones(sys.Nx, 1);
params.distLB_      = -1 * ones(sys.Nx, 1);

params.eps_p_    = 1e-3;
params.eps_d_    = 1e-3;
params.maxIters_ = 8000;
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

%% Scan

data_cvh_rob_noguarantees = cell(length(Ts), 1); 
data_cvh_rob_tset         = cell(length(Ts), 1); 
data_cvh_rob_tcost        = cell(length(Ts), 1); 

for i = 1:length(Ts)
    fprintf('Scanning horizon length %d of %d\n', i, length(Ts));
    params.tFIR_     = Ts(i);

    fprintf('Synthesizing without terminal set...\n')
    data_cvh_rob_noguarantees{i} = run_scenarios(sys, params, tHorizon, w, x0, scenariosRob);
    
    fprintf('Synthesizing with terminal set...\n')
    [params, tStats] = terminal_set(sys, params);
    %params = remove_redundancy_terminal(sys, params);
    data_cvh_rob_tset{i} = run_scenarios(sys, params, tHorizon, w, x0, scenariosRob);
    
    fprintf('Synthesizing with terminal set & terminal cost...\n')
    params.terminal_cost_ = true;
    data_cvh_rob_tcost{i} = run_scenarios(sys, params, tHorizon, w, x0, scenariosRob);
end

name = strcat('cvh_polytopic_seed',num2str(seed),'_ct',num2str(connectThresh*100),'.mat');
save(name);

%% Plot (not formatted for paper)

cvhs_ng = zeros(length(Ts), 1); % No guarantees
cvhs_ts = zeros(length(Ts), 1); % Terminal set
cvhs_tc = zeros(length(Ts), 1); % Terminal cost
for i = 1:length(Ts)
    cvhs_ng(i) = data_cvh_rob_noguarantees{i}{2}{4};
    cvhs_ts(i) = data_cvh_rob_tset{i}{2}{4};
    cvhs_tc(i) = data_cvh_rob_tcost{i}{2}{4};
end

figure(1); hold on;
plot(Ts, cvhs_ng);
hold on
plot(Ts, cvhs_ts);
hold on
plot(Ts, cvhs_tc);

xlabel('T');
ylabel('Runtime per state, polytopic noise');
legend
