addpath('../lib/')
clear; clc;

prob = 0.4;
nn_min = 3;
nn_max = 15;
T = 5;
d = 2;

env_name = 'chain';
iter = 1;   % iter should be higher for grid since topology is random

centralized_tl = zeros(nn_max-nn_min+1, 1);
ddd_tl = cell(nn_max-nn_min+1, 1);
for nn = nn_min:nn_max
    network = nn^2;
    if strcmp(env_name, 'chain')
        [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.chain(network);
    else
        [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.grid(network, prob);
    end
    centralized_tl(nn-nn_min+1) = (Nx + T) * (Nu + 1) - 1;
    for ii=1:iter
        if strcmp(env_name, 'chain')
            [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.chain(network);
        else
            [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.grid(network, prob);
        end
        ddd_tl{nn-nn_min+1} = [ddd_tl{nn-nn_min+1}; ...
            data_driven_lib.find_traj_length(A, p, q, d, T)];
    end
end

%% Save result
save(strcat('../results/tl_net_', env_name));