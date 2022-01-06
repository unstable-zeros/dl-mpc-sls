function results = run_scenarios(sys, params, tHorizon, w, x0, scenarios)
             
    results = cell(length(scenarios), 1);
    infeas  = false;
    for i = 1:length(scenarios)
        scenario = scenarios{i};
        if scenario == ScenarioType.CentNominal
            fprintf('CentNominal======\n');
        elseif scenario == ScenarioType.CentRobust
            fprintf('CentRobust=======\n');
        elseif scenario == ScenarioType.DistNominal
            fprintf('DistNominal======\n');
        elseif scenario == ScenarioType.DistRobust
            fprintf('DistRobust=======\n');
        end
        
        if ~infeas
            % Slightly hacky: we assume that centralized will be run first
            % if centralized reports infeasibility, skip running the
            % subsequent simulations
            [x, u, runtime, infeas] = simulate_scenario(sys, params, tHorizon, w, x0, scenario);   
        else % populate data entries
            x = nan(sys.Nx, tHorizon); 
            u = nan(sys.Nu, tHorizon);
            runtime = 0;            
        end
        
        cost       = get_cost_fn(params, x, u);
        results{i} = {x, u, runtime, cost};
        
    end
end