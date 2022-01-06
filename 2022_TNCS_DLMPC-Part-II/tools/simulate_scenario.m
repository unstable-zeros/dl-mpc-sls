function [x, u, runtime, infeas] = simulate_scenario(sys, params, tHorizon, w, x0, scenario)
% TODO: this is mostly redundant with sls_mpc.m in the main toolbox
% Only difference is we explicitly determine which scenario to use

infeas = false; % warning that centralized shows infeasibility
x      = zeros(sys.Nx, tHorizon); 
u      = zeros(sys.Nu, tHorizon);   
x(:,1) = x0;

times     = zeros(tHorizon-1, 1);

% initially no warm start; will be populated for subsequent timesteps
warmStart = [];

for t=1:tHorizon-1
    fprintf('Calculating time %d of %d\n', t+1, tHorizon);

    if scenario == ScenarioType.DistNominal
        if params.has_coupling() || params.has_terminal_set()
            [~, u(:,t), stats, warmStart] = mpc_coupled_distributed(sys, x(:,t), params, warmStart);
        else
            [~, u(:,t), stats, warmStart] = mpc_uncoupled_distributed(sys, x(:,t), params, warmStart);
        end
        times(t) = stats.time_;

    elseif scenario == ScenarioType.CentNominal
        [~, u(:,t), times(t)] = mpc_centralized(sys, x(:,t), params);
       
    elseif scenario == ScenarioType.DistRobust
        [~, u(:,t), stats, warmStart] = rmpc_distributed(sys, x(:,t), params, warmStart);
        times(t) = stats.time_;
        
    elseif scenario == ScenarioType.CentRobust
        [~, u(:,t), times(t)] = mpc_centralized(sys, x(:,t), params);                
    end

    if any(isnan(u(:,t)))
        % Don't want to stop simulation but want the FYI
        mpc_warning('MPC solver failed/infeasible!');
        infeas = true;
        u(:,t+1:end) = nan;
        x(:,t+1:end) = nan;
        break; % Can't propagate dynamics anyway
    end
    
    % We will ignore the mpc calculated state since it is calculated with no disturbance
    x(:,t+1) = sys.A*x(:,t) + sys.B2*u(:,t) + sys.B1*w(:,t);
end

% Discount first step since that step is not warm-started
runtime = mean(times(2:end));
        
end