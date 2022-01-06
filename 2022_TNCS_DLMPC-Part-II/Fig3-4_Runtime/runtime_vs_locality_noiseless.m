%% Setup plant
gridSize      = 11; % total size = gridSize^2
seed          = 628;
connectThresh = 0.6; % lower = more connected grid

plotTopology = false;

[sys, adjMtx, nodeCoords] = get_plant(gridSize, seed, connectThresh);

if plotTopology
    fig1h = figure(1);
    plot_graph(adjMtx, nodeCoords, 'k');

    x = 10; y = 50; width = 580; height = 500;
    set(fig1h, 'Position', [x y width height]); % display only
end

%% Run simulations
ds       = [4 5 7 10 12];
T        = 5;

data_rvl_nom_offline = cell(length(ds), 1); 

for i = 1:length(ds)
    fprintf('Scanning neighborhood size %d of %d\n', i, length(ds));
    d = ds(i);

    params              = get_controller_params(sys, d, T);
    % Noiseless
    params.distConsMtx_ = [];
    params.distUB_      = [];
    params.distLB_      = [];
        
    [params, tStats] = terminal_set(sys, params);
    data_rvl_nom_offline{i} = [tStats.time_, tStats.iters_];
end

name = strcat('rvl_nom_off_seed',num2str(seed),'_ct',num2str(connectThresh*100),'.mat');
save(name);

%% Plot (not formatted for paper)
rvls = zeros(length(ds), 1);
for i = 1:length(ds)
    rvls(i) = data_rvl_nom_offline{i}(1);    
end

figure(1); hold on;
plot(ds, rvls);

xlabel('d');
title('Runtime terminal set per state, noiseless');