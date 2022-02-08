addpath('../lib/')
%% Parameters
%clc; clear all; close all;
network = 64;
prob = 0.4;
Ts = .1;        % Time step for discretization
T = 5;         % Time horizon (FIR)
Tsim = 30;      % Simulation time

env_name = 'chain';
if strcmp(env_name, 'chain')
    rho = 8000 / network;
else
    rho = 2000 / network;
end

%% Plant dynamics
if strcmp(env_name, 'chain')
    rng(2021);
    [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.chain(network);
else
    rng(2021);
    [Nx, Nu, p, q, A, B, Q, S, Adj] = environment.grid(network, prob);
end
x0 = rand(Nx,1);
N = network;

%% Solve for centralized cost
x_cent = zeros(Nx, Tsim+1);
u_cent = zeros(Nu, Tsim+1);
x_cent(:,1) = x0;
for k = 1:Tsim
    clear LocalityR LocalityM
    xi = x_cent(:, k);
    [X, R, M] = centralized(A, B, p, q, Q, S, T, d, xi);
    % dynamics
    u_cent(:,k) = M(1:Nu, :)*xi;       % control action
    x_cent(:, k+1) = A * xi + B * u_cent(:,k);
end
% Cost 
obj_cent = 0;
for t =1:Tsim
    obj_cent = obj_cent + x_cent(:,t)'*Q*x_cent(:,t)+...
        u_cent(:,t)'*S*u_cent(:,t);
end
obj_cent = obj_cent + x_cent(:,t+1)'*Q*x_cent(:,t+1);
    
%% Solve over increasing locality
costs = zeros(5, 1);

for d=1:5
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

    % Distributed data-driven SLS
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
        [R, M, ti] = ddd(Hx, Hu, Hb, A, B, Q, S, T, d, p, q, xi, rho, ...
            row_mask, x_column_mask, u_column_mask, max_patch_size, Psi_warm);
        t_ddd(k) = ti;
        Psi_warm = [R; M];    
        % dynamics
        u_ddd(:,k) = M(1:Nu, :)*xi;       % control action
        x_ddd(:, k+1) = A * xi + B * u_ddd(:,k);    
    end

    % Cost 
    obj_ddd = 0;
    for t =1:Tsim
        obj_ddd = obj_ddd + x_ddd(:,t)'*Q*x_ddd(:,t)+...
            u_ddd(:,t)'*S*u_ddd(:,t);
    end
    obj_ddd = obj_ddd + x_ddd(:,t+1)'*Q*x_ddd(:,t+1);
    costs(d) = obj_ddd;
end

%% Save data
save(strcat('../results/cost_locality_', env_name))

