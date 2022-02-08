addpath('../lib/')
%% Parameters
clc; clear all; close all;
locality = 2;   % this is between states, not subsystems
prob = 0.4;
Ts = .1;        % Time step for discretization
T = 5;         % Time horizon (FIR)
Tsim = 30;      % Simulation time

% The number of subsystems
networks = [9 16 36 64 81 100 121];
env_per_size = 10;
times = zeros(length(networks), env_per_size);
rng(2020)

exp_env = 'chain';

%% Experiment
for num=1:length(networks)    
    networks(num)
    for ee=1:env_per_size
        if strcmp(exp_env, 'chain')
            [Nx, Nu, p, q, A, B, Q, S] = environment.chain(...
                networks(num));
            rhos = 8000 ./ networks;
        else
            [Nx, Nu, p, q, A, B, Q, S] = environment.grid(...
                networks(num), prob);
            rhos = 2000 ./ networks;
        end
        x0 = rand(Nx,1);
        d = locality;
        
        % Compute masks (as it's common across all env of this size)
        N = networks(num);
        row_mask = cell(N);
        x_column_mask = cell(N);
        u_column_mask = cell(N);
        max_patch_size = 0;
        for i=1:N
            x_row_mask = data_driven_lib.patch_mask(A, d, p, p, i);
            u_row_mask = data_driven_lib.patch_mask(A, d+1, p, q, i);
            row_mask{i} = [repmat(x_row_mask, T, 1); repmat(u_row_mask, T, 1)];
            x_column_mask{i} = data_driven_lib.patch_mask(A, d, p, p, i);
            u_column_mask{i} = data_driven_lib.patch_mask(A, d+1, p, p, i);
            if sum(u_row_mask) > max_patch_size
                max_patch_size = sum(u_row_mask);
            end
        end
    
        % Simulate
        [Hx, Hu, Hb] = data_driven_lib.subsystem_hankel(A, B, p, q, d, T);
        x_ddd = zeros(Nx, Tsim+1);
        u_ddd = zeros(Nu, Tsim+1);
        x_ddd(:,1) = x0;
        Psi_warm = zeros((Nx+Nu)*T, Nx);
        t_ddd = zeros(Tsim, 1);

        Phi_prev = Psi_warm;
        Phi_curr = Psi_warm;
        for k = 1:Tsim
            xi = x_ddd(:, k);
            [R, M, ti] = ddd(Hx, Hu, Hb, A, B, Q, S, T, d, p, q, xi, ...
                rhos(num), row_mask, x_column_mask, u_column_mask, ...
                max_patch_size, Psi_warm);
            t_ddd(k) = ti;
            Psi_warm = [R; M];
            % dynamics
            u_ddd(:,k) = M(1:Nu, :)*xi;       % control action
            x_ddd(:, k+1) = A * xi + B * u_ddd(:,k);    
        end
        times(num, ee) = mean(t_ddd(2:end));
        mean(t_ddd(2:end))
    end
end

%% Save data
save(strcat('../results/scalability_', exp_env));



