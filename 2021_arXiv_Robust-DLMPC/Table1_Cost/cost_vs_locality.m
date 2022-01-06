%% Run simulation
N        = 10;
ds       = [4];
T        = 5;
seed     = 2021;
tHorizon = 20;

numDists = 5; % 5 different disturbances
numSims  = length(ds) * numDists; % total number of simulations

scenariosNom = {ScenarioType.CentNominal, ScenarioType.DistNominal};
scenariosRob = {ScenarioType.CentRobust, ScenarioType.DistRobust};

[sys, w, x0] = setup_plant(N, tHorizon, seed);

data_cvl_nom = cell(numSims, 1);
data_cvl_rob = cell(numSims, 1);

for i = 1:length(ds)
    fprintf('Scanning neighborhood size %d of %d\n', i, length(ds));
    d = ds(i);

    paramsNom = setup_ctrller_partial(sys, d, T);
    
    ub = 20 * ones(sys.Nx, 1);
    ub([1,3,5,6,8,10]) = 1.5;
    paramsNom.stateUB_  = ub;
    paramsNom.stateLB_  = -ub;
        
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
    
    for j = 1:numDists
        idx = (i-1)*numDists + j;
        
        % random disturbance and initial value
        w  = 2 * rand(sys.Nx, tHorizon) - 1;
        x0 = rand(sys.Nx, 1);

        if j == 2
            paramsNom.rhoMax_ = 1; % help nominal converge            
        end

        if j == 3
            rng(2022); w = 2 * rand(sys.Nx, tHorizon) - 1;
        elseif j == 4
            rng(2021); 
            w = 2 * rand(sys.Nx, tHorizon) - 1;
            w = 2 * rand(sys.Nx, tHorizon) - 1;
            w = 2 * rand(sys.Nx, tHorizon) - 1;
            w = 2 * rand(sys.Nx, tHorizon) - 1;
        elseif j == 5
            rng(1993);
            w = 2 * rand(sys.Nx, tHorizon) - 1;            
        end    

        data_cvl_nom{idx} = run_scenarios(sys, paramsNom, tHorizon, w, x0, scenariosNom);
        data_cvl_rob{idx} = run_scenarios(sys, paramsRob, tHorizon, w, x0, scenariosRob);
    end
end

save('data_cvl.mat');

%% Output / plot (not formatted for paper)
cvls = zeros(length(ds), 4);

numDists = 1;
for i = 1:length(ds)
    for j = 1:numDists
        idx = (i-1)*numDists + j;
        
        % sum costs
        cvls(i, 1) = cvls(i, 1) + data_cvl_nom{idx}{1}{4}; % cent nominal
        cvls(i, 2) = cvls(i, 2) + data_cvl_rob{idx}{1}{4}; % cent robust
        cvls(i, 3) = cvls(i, 3) + data_cvl_nom{idx}{2}{4}; % dist nominal
        cvls(i, 4) = cvls(i, 4) + data_cvl_rob{idx}{2}{4}; % dist robust
    end
end

cvls = cvls ./ numDists; % take average cost over cases with different disturbances

% Print data as 'table'
ds
nominalCent = cvls(:, 1)'
robustCent  = cvls(:, 2)'
nominalDist = cvls(:, 3)'
robustDist  = cvls(:, 4)'
