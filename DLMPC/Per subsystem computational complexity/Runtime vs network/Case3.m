%% Algorithm II

clc; clear all; close all;

% parpool

index = 0;
cases = [10 50 100 200];

for network = cases
    
    index = index+1; time(index) = 0; time_centr(index) = 0; iter(index) = 0; clear x u x_VAL u_VAL LocalityR LocalityM X
    
    locality = 3;
    
%% Plant dynamics

% % Number of pendulums
n = network;

Nx = 2*n; Nu = n;

% A matrix
Ac = zeros(Nx,Nx); j = 0;
for i = 1:2:Nx
    j = j+1;
    if j == 1
        Ac (i:i+1,i+2:i+3) = [0    0; 1  1];
        Ac (i:i+1,i:i+1) = [0 1; -3 -3];
    elseif j == Nx/2        
        Ac (i:i+1,i:i+1) = [0 1; -3 -3];
        Ac (i:i+1,i-2:i-1) = [0    0; 1 1];
    else
        Ac (i:i+1,i+2:i+3) = [0    0; 1 1];
        Ac (i:i+1,i:i+1) = [0 1; -3 -3];
        Ac (i:i+1,i-2:i-1) = [0    0; 1 1];
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

% Time horizonn (FIR)
T = 5;

% Locality constraint
d = 3;

% Initial condition
rng(2020)
x0 = rand(Nx,1);

% Simulation time
Tsim = 10;

%% Feasibility constraints

E1 = [eye(Nx);zeros(Nx*(T-1),Nx)];

I = kron(eye(T),eye(Nx));

Z = kron(eye(T-1),eye(Nx));
Z = [zeros(Nx,Nx*(T));Z,zeros(Nx*(T-1),Nx)];

tmp = repmat({A},T,1);
Aa = blkdiag(tmp{:});
clear tmp
tmp = repmat({B},T,1);
Bb = blkdiag(tmp{:});

IZAa = I - Z*Aa;
ZB = -Z*Bb;

IZA_ZB = [IZAa ZB];
IZA_ZB = IZA_ZB(:,1:end-Nu);
invIZA_ZB = inv(IZA_ZB*IZA_ZB');

%% Coupling weights and constraints

Q = diag(ones(Nx,1))+diag(-1/2*ones(Nx-2,1),2)+diag(-1/2*ones(Nx-2,1),-2);
S = diag(ones(Nu,1));

% Build the big cost matrix
C = [];
for t = 0:T-1
    C =  [C; zeros(Nx,t*Nx) Q zeros(Nx,(T-t-1)*Nx+(T-1)*Nu)];
end
for t = 0:T-2
    C =  [C;zeros(Nu,T*Nx+t*Nu) S zeros(Nu,(T-t-2)*Nu)];
end

% Coupling
for i = 1:Nx*T+Nu*(T-1)
    indeces{i} = [];
    for j = 1:Nx*T+Nu*(T-1)
        if C(i,j) ~= 0
            indeces{i} = [indeces{i} j];
        end
        indeces{i} = unique(indeces{i});
    end
end

%% Locality constraints

Comms_Adj = abs(A)>0;
for t = 1:T
    LocalityR{t} = Comms_Adj^(d-1)>0;
    LocalityM{t} = abs(B)'*LocalityR{t}>0;
end

% Separate by columns (see columnwise_separability.m for details)
for i = 1:Nx
    c{i} = i;
    count = 0;
    for j = 1:T+(T-1)
        if j<=T
            find_locR = find(LocalityR{j}(:,i));
            for k =1:max(length(find_locR))
                count = count +1;
                s_c{i}(count) = find_locR(k)+(j-1)*Nx;
                if j == T
                    s_c_T{i}(k) = count;
                end
            end
        else
            find_locM = find(LocalityM{j-T}(:,i));
            for k =1:max(length(find_locM))
                count = count +1;
                s_c{i}(count) = find_locM(k)+(j-T-1)*Nu+T*Nx;
            end
        end
    end
end

% Separate by rows (see rowwise_separability.m for details)
k = 0;
for i = 1:Nx
    if mod(i, Nx/Nu) == 0 % Decide whether or not there is actuation
        s_r{i} = zeros(T+(T-1),Nx); % Prealocate the indices
        k = k+1;
        for j = 1:T+(T-1)
            if j<=T
                r{i}(j) = Nx*(j-1) + i;
                s_r{i}(j,1:max(length(find(LocalityR{j}(i,:))))) = find(LocalityR{j}(i,:));
            else
                r{i}(j) = Nu*(j-T-1) + Nx*T + k;
                s_r{i}(j,1:max(length(find(LocalityM{j-T}(k,:))))) = find(LocalityM{j-T}(k,:));
            end
        end
    else
        s_r{i} = zeros(T,Nx); % Prealocate the indices
        for j = 1:T
            r{i}(j) = Nx*(j-1) + i;
            s_r{i}(j,1:max(length(find(LocalityR{j}(i,:))))) = find(LocalityR{j}(i,:));
        end
    end
    s_r{i}( :, ~any(s_r{i},1) ) = []; % Eliminate the columns with only zeros
end

%% ADMM

%% Syntheize the controller

x(:,1) = x0;
xi = x0;

% Warm-start
Phi = zeros(Nx*T + Nu*(T-1),Nx);
Psi = zeros(Nx*T + Nu*(T-1),Nx);
Lambda = zeros(Nx*T + Nu*(T-1),Nx);
for i = 1:Nx*T+Nu*(T-1)
    if isempty(indeces{i}) == 0
        Z_admm{i} = 0;
        for j = indeces{i}
            Y{i}{j} = 0;
        end
    else
    end
end

for t = 1:Tsim
    
Psi_prev = ones(Nx*T + Nu*(T-1),Nx); % Just so the while doesn't break

rho = 1;

count = 0;conv = [1];
while norm(conv) ~= 0 
    
Psi_prev = Psi;

        %% Row-wise separability
        
        % Separate the given matrices
k = 0;
for sys = 1:Nx
    if mod(sys, Nx/Nu) == 0
        k = k+1;
    end
    for i = r{sys}
        j = find(r{sys}==i);
        if j<=T
            Psi_loc_row{i} = Psi(i,s_r{sys}(j,1:max(length(find(LocalityR{j}(sys,:))))));
            Lambda_loc_row{i} = Lambda(i,s_r{sys}(j,1:max(length(find(LocalityR{j}(sys,:))))));
        else
            Psi_loc_row{i} = Psi(i,s_r{sys}(j,1:max(length(find(LocalityM{j-T}(k,:))))));
            Lambda_loc_row{i} = Lambda(i,s_r{sys}(j,1:max(length(find(LocalityM{j-T}(k,:))))));
        end
    end
end


%% ADMM
counter = 0; conv1 = 1; conv2 = 1;

mu = 1;

while norm(conv1)>10^(-3) || norm(conv2)>10^(-4)
    
    
    %% Phi & X
    
    sys = 1; % Separate the first row since we want to measure time
        tic;
        
        for i = r{sys}
            
            n = max(length(s_r{sys}(find(r{sys}==i),:)));
            
            % Define parameters
            m_j = max(length(indeces{i}));
            C_proxi = C(i,indeces{i});
            M1 = [eye(n) zeros(n,m_j-1)];
            Id = eye(m_j-1);
            xi_i = xi(s_r{sys}(find(r{sys}==i),:));
            
            i_new = find(indeces{i} == i);
            
            M2 = [zeros(i_new-1,n) Id(1:i_new-1,:); xi_i' zeros(1,m_j-1); zeros(m_j-i_new,n) Id(i_new:end,:)];
            a = Psi_loc_row{i}-Lambda_loc_row{i};
            
            Mj_sum = 0; Mjb_sum = 0;
            for j = 1:m_j
                
                if j < i_new
                    M{j} = zeros(1,m_j+n-1); M{j}(n+j) = 1;
                elseif j == i_new
                    M{j} = [xi_i' zeros(1,m_j-1)];
                elseif j>i_new
                    M{j} = zeros(1,m_j+n-1); M{j}(n+j-1) = 1;
                end
                
                Mj_sum = Mj_sum + (M{j}'*M{j});
                
                k = indeces{i}(j);
                b = Z_admm{k}-Y{i}{k};
                Mjb_sum = Mjb_sum + M{j}'*b;
            end
            
            W = pinv(2*((C_proxi*M2)'*C_proxi*M2)+rho*(M1'*M1)+mu*Mj_sum)*(rho*M1'*a'+mu*Mjb_sum);
            
            
            Phi_loc{i} = (M1*W)';
            
            X_i = M2*W;
            X{i} = zeros(Nx*T+Nu*(T-1),1); X{i}(indeces{i}) = X_i;
            
        end
        
        [~] = toc;
        time(index) = time(index) + toc;
    
    for sys = 2:Nx
        
        for i = r{sys}
            
            n = max(length(s_r{sys}(find(r{sys}==i),:)));
            
            % Define parameters
            m_j = max(length(indeces{i}));
            C_proxi = C(i,indeces{i});
            M1 = [eye(n) zeros(n,m_j-1)];
            Id = eye(m_j-1);
            xi_i = xi(s_r{sys}(find(r{sys}==i),:));
            
            i_new = find(indeces{i} == i);
            
            M2 = [zeros(i_new-1,n) Id(1:i_new-1,:); xi_i' zeros(1,m_j-1); zeros(m_j-i_new,n) Id(i_new:end,:)];
            a = Psi_loc_row{i}-Lambda_loc_row{i};
            
            Mj_sum = 0; Mjb_sum = 0;
            for j = 1:m_j
                
                if j < i_new
                    M{j} = zeros(1,m_j+n-1); M{j}(n+j) = 1;
                elseif j == i_new
                    M{j} = [xi_i' zeros(1,m_j-1)];
                elseif j>i_new
                    M{j} = zeros(1,m_j+n-1); M{j}(n+j-1) = 1;
                end
                
                Mj_sum = Mj_sum + (M{j}'*M{j});
                
                k = indeces{i}(j);
                b = Z_admm{k}-Y{i}{k};
                Mjb_sum = Mjb_sum + M{j}'*b;
            end
            
            W = pinv(2*((C_proxi*M2)'*C_proxi*M2)+rho*(M1'*M1)+mu*Mj_sum)*(rho*M1'*a'+mu*Mjb_sum);
            
            
            Phi_loc{i} = (M1*W)';
            
            X_i = M2*W;
            X{i} = zeros(Nx*T+Nu*(T-1),1); X{i}(indeces{i}) = X_i;
            
        end
    end
    
    %% Z
    
    Z_old = Z_admm;
    
    sys = 1; % Separate since we want to measure time
    tic;
    for j = r{sys}
        Z_admm{j} = 0;
        for i = indeces{j}
            Z_admm{j} = Z_admm{j} + (X{i}(j)+Y{i}{j})/length(indeces{j});
        end
    end
    [~] = toc;
    time(index) = time(index) + toc;
    
    for sys = 2:Nx
        for j = r{sys}
            Z_admm{j} = 0;
            for i = indeces{j}
                Z_admm{j} = Z_admm{j} + (X{i}(j)+Y{i}{j})/length(indeces{j});
            end
        end
    end
   
    %% Lagrange multiplier
    
    sys = 1; % Separate since we want to measure time
    tic;
    for i = r{sys}
        for j = indeces{i}
            Y{i}{j} = Y{i}{j} + X{i}(j) - Z_admm{j};
        end
    end
    [~] = toc;
    time(index) = time(index) + toc;
    
    for sys = 2:Nx
        for i = r{sys}
            for j = indeces{i}
                Y{i}{j} = Y{i}{j} + X{i}(j) - Z_admm{j};
            end
        end
    end
    
    %% Convergence criterium
    
    % Primal residue
    for sys = 1:Nx
        for i = r{sys}
            average = zeros(Nx*T+Nu*(T-1),1);
            for j = indeces{i}
                average(j) = Z_admm{j};
            end
            primal (i) = norm(X{i}-average);
        end
    end
    conv1 = max(primal); 

    % Dual residue 
    for sys = 1:Nx
        for j = r{sys}
            dual(j) = norm(Z_admm{j}-Z_old{j});
        end
    end
    conv2 = max(dual); 
    
    counter = counter + 1;
    if counter > 500
        break
        disp('Consensus ADMM did not converge')
    end

end

    if t > 1
        iter(index) = iter(index) + counter;
    end

% Build the big matrix
for sys = 1:Nx
    for i = r{sys}
        Phi(i,s_r{sys}(find(r{sys}==i),:)) = Phi_loc{i};
    end
end

    %% Column-wise separability
        % Separate the given matrices
        for i = 1:Nx
            Phi_loc_col{i} = Phi(s_c{i},c{i});
            Lambda_loc_col{i} = Lambda(s_c{i},c{i});
        end
        
        % Solve for each column
        i = 1; % Separate the first row since we want to measure time
        IZA_ZB_loc = IZA_ZB(:,s_c{i}); row_all_zeros = find(all(IZA_ZB_loc == 0,2)); keep_indices = setdiff(linspace(1,Nx*T,Nx*T),row_all_zeros);
        IZA_ZB_loc = IZA_ZB(keep_indices,s_c{i}); E1_loc = E1(keep_indices,c{i}); 
        tic;
        AUX_matrix = IZA_ZB_loc'*pinv(IZA_ZB_loc*IZA_ZB_loc');
        Psi_loc{i} = (Phi_loc_col{i}+Lambda_loc_col{i})+AUX_matrix*(E1_loc-IZA_ZB_loc*(Phi_loc_col{i}+Lambda_loc_col{i}));
        [~] = toc;
        time(index) = time(index) + toc;

        for i = 2:Nx
            clear AUX_matrix
            IZA_ZB_loc = IZA_ZB(:,s_c{i}); row_all_zeros = find(all(IZA_ZB_loc == 0,2)); keep_indices = setdiff(linspace(1,Nx*T,Nx*T),row_all_zeros);
            IZA_ZB_loc = IZA_ZB(keep_indices,s_c{i}); E1_loc = E1(keep_indices,c{i}); 
            AUX_matrix = IZA_ZB_loc'*pinv(IZA_ZB_loc*IZA_ZB_loc');
            Psi_loc{i} = (Phi_loc_col{i}+Lambda_loc_col{i})+AUX_matrix*(E1_loc-IZA_ZB_loc*(Phi_loc_col{i}+Lambda_loc_col{i}));
        end
         
        % Build the big matrix
        for i = 1:Nx
            Psi(s_c{i},c{i}) = Psi_loc{i};
        end

    %% Lagrange multiplier
    Lambda = Lambda + Phi - Psi;

    %% Convergence
        % Local convergence criterium
        conv = [0];
        
        for sys = 1:Nx
            local_phi = Phi(r{sys},s_r{sys}(T,:));
            local_psi = Psi(r{sys},s_r{sys}(T,:));
            local_psi_prev = Psi_prev(r{sys},s_r{sys}(T,:));

            local_conv1 = norm(local_phi-local_psi,'fro');
            local_conv2 = norm(local_psi-local_psi_prev,'fro');
            
            if local_conv1 > 10^(-4) || local_conv2 > 10^(-3)
                 conv = [conv 1];
            end
        end
        
        % Number of iterations until convergence
        count = count + 1
        if count > 10000
            disp ('ADMM did not converge')
            break
        end

end

    if t > 1
        iter(index) = iter(index) + count;
    end

%% Dynamics
    
    % Compute the control action (in a localized way)
    u(:,t) = Phi(1+Nx*T:Nx*T+Nu,:)*xi;
    
    % Simulate what the dynamics are given that action
    x(:,t+1) = Phi(1+Nx:2*Nx,:)*xi; % Since there is no noise x_ref = x
    
    % Update the initial condition
    xi = x(:,t+1);
    
end
time(index) = time(index)/(Tsim-1);


%% Validation

x_VAL(:,1) = x0;
xi = x0;

for k = 1:Tsim
    
    clear LocalityR LocalityM
    
    Comms_Adj = abs(A)>0;
    LocalityR = Comms_Adj^(d-1)>0;
    
    count = 0;
    for t = 1:T
        Rsupport{t} = LocalityR>0;
        Msupport{t} = (abs(B)'*Rsupport{t})>0;
        count = count + sum(sum(Rsupport{t}))+sum(sum(Msupport{t}));
    end
    
%     tic
    cvx_begin
    cvx_precision low
    
    cvx_solver_settings('dumpfile','file2getruntime')
    
    variable X(count)
    expression Rs(Nx,Nx,T)
    expression Ms(Nu,Nx,T)
    
    % Populate decision variables
    % Locality constraints automatically enforced by limiting support of R and M
    spot = 0;
    for t = 1:T
        R{t} = Rs(:,:,t);
        supp = find(Rsupport{t});
        num = sum(sum(Rsupport{t}));
        R{t}(supp) = X(spot+1:spot+num);
        spot = spot + num;
        
        M{t} = Ms(:,:,t);
        supp = find(Msupport{t});
        num = sum(sum(Msupport{t}));
        M{t}(supp) = X(spot+1:spot+num);
        spot = spot + num;
    end
    
    % Set up objective function
    objective = 0;
    for t = 1:T
        vect = vec([Q zeros(Nx,Nu); zeros(Nu,Nx) S]*[R{t};M{t}]*xi);
        objective = objective + vect'*vect;
    end
    
    % Perform minimization
    minimize(objective)
    subject to
    % Achievability constraints
    R{1} == eye(Nx);
    for t= 1:T-1
        R{t+1} == A*R{t} + B*M{t};
    end
    cvx_end
%     [~] = toc;
%     time_centr(index) = time_centr(index) + toc;

    if t>1
        load('file2getruntime.mat')
        runtime = getfield(res,'runtime');
        time_centr(index) = time_centr(index) + runtime;
    end
    
    %% Dynamics
    
    % Compute the control action
    u_VAL(:,k) = M{1}*xi;
    
    % Simulate what the dynamics are given that action
    x_VAL(:,k+1) = R{2}*xi; % Since there is no noise x_ref = x
    
    % Update the initial condition
    xi = x_VAL(:,k+1); 
    
end
time_centr(index) = time_centr(index)/(Tsim-1);

end

%% Plot

figure (1)
subplot(1,2,1)
plot(cases,time,'m-s','LineWidth',2)
hold on
plot(cases,time_centr,'b-s','LineWidth',2)
xlabel('$$Number\ of\ pendulums\ in\ the\ network$$','Interpreter','latex','Fontsize', 16)
ylabel('$$Average\ runtime\ per\ MPC\ iteration\ for\ each\ state\ (seconds)$$','Interpreter','latex','Fontsize', 16)
leg4 = legend('$$Localized\ ADMM\ Solution$$', '$$Centralized\ Solution$$');
set(leg4,'Interpreter','latex','Fontsize', 12);

subplot(1,2,2)
plot(cases,iter,'m-s','LineWidth',2)
xlabel('$$Number\ of\ pendulums\ in\ the\ network$$','Interpreter','latex','Fontsize', 16)
ylabel('$$Average\ number\ of\ ADMM\ iterations\ per\ MPC\ iteration\ for\ each\ state\ (seconds)$$','Interpreter','latex','Fontsize', 16)

