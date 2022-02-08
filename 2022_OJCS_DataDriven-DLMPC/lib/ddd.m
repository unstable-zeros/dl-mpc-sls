function [R, M, ti] = ddd(Hx, Hu, Hb, A, B, Q, R, T, d, p, q, xi, rho, ...
    row_mask, x_column_mask, u_column_mask, max_patch_size, Psi_warm)
    [Nx, Nu] = size(B);
    N = Nx / p;
    
    eps_p = 6e-3;
    eps_d = 6e-3;
    max_iter = 100;
    X = zeros(max_iter, 1);
    
    time = 0;
    
    % Scale the termination condition based on patch size
    eps_p = eps_p * max_patch_size;
    eps_d = eps_d * max_patch_size;
        
    % Initialize the variables
    Phi = zeros((Nx+Nu)*T, Nx);
    Psi = Psi_warm;
    Psi_prev = Psi_warm;
    Lbd = zeros(size(Phi));
    
    % big loop until convergence
    for it=1:max_iter+1
        if it == max_iter+1
           disp('did not converge');
           break;
        end
        
        % Compute Phi (using a loop over subsystems)
        for i=1:N
            % Row masks for the given subsystem
            x_subsys_row = false((Nx+Nu)*T, 1);
            x_subsys_row(1:Nx*T) = repmat(...
                data_driven_lib.subsys_mask(Nx, p, i), T, 1);
            u_subsys_row = false((Nx+Nu)*T, 1);
            u_subsys_row(Nx*T+1:end) = repmat(...
                data_driven_lib.subsys_mask(Nu, q, i), T, 1);
            
            tic;
            % Using Gurobi Optimizer
            % Solve for Phi_x
            I_x = kron(eye(p*T), xi(x_column_mask{i})');
          
            model.Q = sparse( I_x' * I_x + rho/2*eye(size(I_x, 2)) );
            model.obj = -rho * vec( Psi(x_subsys_row, x_column_mask{i})'-...
                Lbd(x_subsys_row, x_column_mask{i})' );            
            model.A = sparse( zeros(size(I_x, 2)) );
            model.rhs = zeros(size(I_x, 2), 1);
            model.lb = -1e2 * ones(size(I_x, 2), 1);
            model.sense = '=';
            params.outputflag = 0;
            results = gurobi(model, params);
            Px_gurobi = reshape(results.x, sum(x_column_mask{i}), p*T)';
            
            
            % Solve for Pu
            I_u = kron(eye(q*T), xi(u_column_mask{i})');
            model.Q = sparse( I_u' * I_u + rho/2*eye(size(I_u, 2)) );
            model.obj = -rho * vec( Psi(u_subsys_row, u_column_mask{i})' -...
                Lbd(u_subsys_row, u_column_mask{i})' );
            model.A = sparse( zeros(size(I_u, 2)) );
            model.rhs = zeros(size(I_u, 2), 1);
            model.lb = -1e3 * ones(size(I_u, 2), 1);
            params.outputflag = 0;
            results = gurobi(model, params);
            
            Pu_gurobi = reshape(results.x, sum(u_column_mask{i}), q*T)';
            Phi(x_subsys_row, x_column_mask{i}) = Px_gurobi;
            Phi(u_subsys_row, u_column_mask{i}) = Pu_gurobi;
            
            time = time + toc;
        end
        
        % Compute Psi (using a loop over subsystems)
        for i=1:N
            subsys_cols = data_driven_lib.subsys_mask(Nx, p, i);
            Hi = [Hx{i}; Hu{i}];
            rr = row_mask{i}(1:Nx);
            id = eye(sum(rr));
            in_patch_mask = and(rr, subsys_cols);
            in_patch_mask = in_patch_mask(rr==1);

            tic;            
            % Solve for Psi in closed form
            Htilde = kron(Hi, eye(p));
            PL = Phi(row_mask{i}, subsys_cols) + Lbd(row_mask{i}, subsys_cols);  
            bb = 2 * Htilde' * vec(PL');
            Atilde = kron([Hi(1:sum(rr), :); Hb{i}], eye(p));
            rhs = vec([id(:, in_patch_mask);zeros(size(Hb{i},1),p)]');
            mat_lhs = [2*(Htilde'*Htilde) Atilde'; ...
                       Atilde zeros(size(Atilde, 1))];
            mat_rhs = [bb; rhs];
            gl = pinv(mat_lhs) * mat_rhs;
            ggg = reshape(gl(1:p*size(Hx{i}, 2)), p, size(Hx{i}, 2))';
            Psi(row_mask{i}, subsys_cols) = Hi * ggg;
            
            time = time + toc;
            
        end
        
        % Compute Lambda
        Lbd = Lbd + Phi - Psi;
        
        % Check whether converged; bookkeeping
        max_p_gap = 0;
        max_d_gap = 0;
        for i=1:N
            subsys_cols = data_driven_lib.subsys_mask(Nx, p, i);
            pi = norm(Phi(row_mask{i}, subsys_cols) - ...
                Psi(row_mask{i}, subsys_cols), 'fro');
            di = norm(Psi(row_mask{i}, subsys_cols) - ...
                Psi_prev(row_mask{i}, subsys_cols), 'fro');
            if pi > max_p_gap
                max_p_gap = pi;
            end
            if di > max_d_gap
                max_d_gap = di;
            end
        end
        
        fprintf("Iteration: %d\ts: %d\t d: %d\t time:%d\n", it, ...
            max_p_gap, max_d_gap, time/N);
        
        if max_p_gap < eps_p && max_d_gap < eps_d            
            break
        end
        
        Psi_prev = Psi;
    end
    
    R = Psi(1:Nx*T,:);
    M = Psi(Nx*T+1:end,:);
    ti = time/N;
end