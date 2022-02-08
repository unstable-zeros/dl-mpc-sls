function [G, time] = data_opt(xhank, uhank, A, B, Q, S, T, d, p, q, xi, G_warm)
    [Nx, Nu] = size(B);
    global_H = [xhank; uhank];
    network = Nx / p;
    time = 0;
    
    tic;
    cvx_begin quiet
    cvx_precision low
    variable G(size(global_H, 2), Nx)
    
    % Objective
    vec_xu = vec(global_H * G * xi);
    objective = vec_xu' * vec_xu;
    minimize(objective);
    
    % Constraints
    subject to   
    network = Nx / p;
    for i=1:network
        
        % Achievability constraint for this subsystem
        x_row_mask = data_driven_lib.subsys_mask(Nx, p, i);
        global_I = eye(Nx);
        xhank(x_row_mask, :) * G == global_I(x_row_mask, :);
        
        % Locality of state response
        x_patch_mask = data_driven_lib.patch_mask(A, d, p, p, i);
        u_patch_mask = data_driven_lib.patch_mask(A, d+1, p, q, i);
        x_mask = repmat(x_patch_mask, T, 1);
        x_mask(1:Nx) = 1;        
        u_mask = repmat(u_patch_mask, T, 1);
        xhank(~x_mask, :) * G(:, data_driven_lib.subsys_mask(Nx, p, i)) == 0;
        uhank(~u_mask, :) * G(:, data_driven_lib.subsys_mask(Nx, p, i)) == 0;
        
    end
    cvx_end
    time = toc / network;
    
%     Q_rt = kron(xi', global_H);
%     model.Q = sparse( Q_rt' * Q_rt );
%     A_gurobi = sparse( kron(eye(Nx), xhank(1:Nx, :)) );
%     I_g = eye(Nx);
%     rhs = vec(I_g);
%     
%     for i=1:network
%         ss_mask = p*(i-1)+1 : (p*i);
%         Ei = I_g(:, ss_mask);
%         
%         x_patch_mask = data_driven_lib.patch_mask(A, d, p, p, i);
%         u_patch_mask = data_driven_lib.patch_mask(A, d+1, p, q, i);
%         
%         x_mask = repmat(x_patch_mask, T, 1);
%         x_mask(1:Nx) = 1;
%         u_mask = repmat(u_patch_mask, T, 1);
%         Hb = [xhank(~x_mask, :); uhank(~u_mask, :)];
%         
%         A_gurobi = [A_gurobi; kron(Ei', Hb)];
%         rhs = [rhs; zeros(size(Hb, 1) * p, 1)];
%     end
%     model.A = sparse(A_gurobi);
%     model.rhs = rhs;
%     model.lb = -1e2 * ones(size(model.Q, 1), 1);
%     model.sense = '=';
%     model.pstart = vec(G_warm);    
%     params.outputflag = 0;
%     
%     tic;
%     results = gurobi(model, params);
%     time = toc / network;
%     
%     G = reshape(results.x, size(xhank, 2), Nx);
end



























