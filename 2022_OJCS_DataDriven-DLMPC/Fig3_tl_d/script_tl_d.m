addpath('../lib/')
clear; clc;

prob = 0.4;
d_min = 1;
d_max = 5;
network = 64;
T = 5;

env_name = 'chain';
iter = 1;
d_tl = zeros(d_max-d_min+1, network * iter);

for ii=1:iter
    if strcmp(env_name, 'chain')
        rng(2021);
        [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.chain(network);
    else
        rng(2021);
        [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.grid(network, prob);
    end
    for d=d_min:d_max
        d_tl(d-d_min+1, (ii-1)*network+1:ii*network) = ...
            data_driven_lib.find_traj_length(A, p, q, d, T);
    end
end

%% Save data
save(strcat('../results/tl_d_', env_name));