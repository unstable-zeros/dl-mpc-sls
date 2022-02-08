function [X, R, M] = centralized(A, B, p, q, Q, S, T, d, xi)
    [Nx, Nu] = size(B);
    
    cvx_begin quiet
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
    cvx_end
    
    X = 0;
    R = Px;
    M = Pu;
end



























