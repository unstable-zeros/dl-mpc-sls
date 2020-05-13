%% Algorithm II

clc; clear all; close all;

locality = 3; network = 4;
    
%% Plant dynamics

% Number of pendulums
n = network;

Nx = 2*n; Nu = n;

% A matrix
m = 1; k = 1; d = 3; g = 10; l = 1;

block_off_diag = [0    0; k*l/m  d/(m*l)];
block_diag_extr = [0 1; -g-k*l/m -d/(m*l)];
block_diag = [0 1; -g-2*k*l/m -2*d/(m*l)];

Ac = zeros(Nx,Nx); j = 0;
for i = 1:2:Nx
    j = j+1;
    if j == 1 % first node
        Ac (i:i+1,i+2:i+3) = block_off_diag;
        Ac (i:i+1,i:i+1) = block_diag;
    elseif j == Nx/2  % last node      
        Ac (i:i+1,i:i+1) = block_diag;
        Ac (i:i+1,i-2:i-1) = block_off_diag;
    else
        Ac (i:i+1,i+2:i+3) = block_off_diag;
        Ac (i:i+1,i:i+1) = block_diag;
        Ac (i:i+1,i-2:i-1) = block_off_diag;
    end
end

% B matrix
Bc = zeros(Nx,Nu); j = 0;
for i = 1:2:Nx
    j = j+1;
    Bc (i:i+1,j) = [0; 1];
end

% Discretize 
Ts = .1;

A  = (eye(Nx)+Ac*Ts);
B = Ts*Bc;

%% Scenario definition

% Locality constraint
d = locality;

% Initial condition
rng(2020)
x0 = rand(Nx,1);

% Simulation time
Tsim = 50;

%% Dynamics

x_VAL(:,1) = x0;
for t = 1:Tsim
    x_VAL(:,t+1) = A*x_VAL(:,t);
end

figure(2)
plot(1:Tsim+1,x_VAL(1,:),1:Tsim+1,x_VAL(3,:))