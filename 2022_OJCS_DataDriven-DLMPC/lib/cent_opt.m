function [X, R, M] = cent_opt(A, B, p, q, Q, S, T, d, xi)
    [Nx, Nu] = size(B);
    
    cvx_begin quiet
    cvx_precision low
    variable Px(Nx*T, Nx)
    variable Pu(Nu*T, Nx)
    
    % Objective
    vec_xu = vec([Px; Pu] * xi);
    objective = vec_xu' * vec_xu;
    minimize(objective);
    
    % Constraints
    subject to
    Px(1:Nx, :) == eye(Nx);
    for t=1:T-1
        Px(1+Nx*t : Nx*(t+1), :) == ...
            A*Px(1+Nx*(t-1) : Nx*t, :) + B * Pu(1+Nu*(t-1) : Nu*t, :);
    end
    
    network = Nx / p;
    for i=1:network        
        % Locality of state response
        x_patch_mask = data_driven_lib.patch_mask(A, d, p, p, i);
        u_patch_mask = data_driven_lib.patch_mask(A, d+1, p, q, i);
        x_mask = repmat(x_patch_mask, T, 1);
        x_mask(1:Nx) = 1;
        u_mask = repmat(u_patch_mask, T, 1);
        Px(~x_mask, data_driven_lib.subsys_mask(Nx, p, i)) == 0;
        Pu(~u_mask, data_driven_lib.subsys_mask(Nx, p, i)) == 0;
        
        % Start Debug
%         x_bb_mask = xor(data_driven_lib.patch_mask(A, d+2, p, p, i),...
%             x_patch_mask);
%         u_bb_mask = xor(data_driven_lib.patch_mask(A, d+2, p, q, i),...
%             u_patch_mask);
%         x_bb_mask = repmat(x_bb_mask, T, 1);
%         x_bb_mask(1:Nx) = 0;
%         u_bb_mask = repmat(u_bb_mask, T, 1);
%         Px(x_bb_mask, data_driven_lib.subsys_mask(Nx, p, i)) == 0;
%         Pu(u_bb_mask, data_driven_lib.subsys_mask(Nx, p, i)) == 0;
        % End Debug
    end
    cvx_end
    
    X = 0;
    R = Px;
    M = Pu;
end



























