classdef environment
    methods     ( Static = true )
        function [Nx, Nu, p, q, A, B, Q, S] = pendulum(n, Ts)
            p = 2;
            q = 1;
            Nx = p*n; Nu = q*n;

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
            A  = (eye(Nx)+Ac*Ts);
            B = Ts*Bc;


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Coupling weights and constraints
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Set-point (scenarios 1 & 2)
            Q = eye(Nx);
            S = diag(ones(Nu,1));
        end
        function [Nx, Nu, p, q, A, B, Q, S] = scaler_sys(n, Anorm)
            p = 1;
            q = 1;
            Nx = p*n; Nu = q*n;

            % A matrix: randomly sample a banded matrix (discrete-time A).
            A = zeros(Nx, Nx);
            A(1, 1:2) = rand(1,2) - 0.5;
            for i = 2:Nx-1
                A(i, i-1:i+1) = rand(1,3) - 0.5;
            end
            A(Nx, Nx-1:Nx) = rand(1,2) - 0.5;
            A = A / max(abs(eig(A))) * Anorm;

            B = eye(Nx);
            Q = eye(Nx);
            S = diag(ones(Nu,1));
        end
        function [Nx, Nu, p, q, A_dyn, B_dyn, Q, S, A] = chain(N)
            Nx = 2*N;
            Nu = N;
            Q = eye(Nx);
            S = eye(Nu);
            p = 2;
            q = 1;
            
            A = zeros(N);
            dt = 0.2;

            % Sample an adjacency matrix for the graph
            for i=1:N-1
                A(i, i+1) = 1;
                A(i+1, i) = 1;
            end

            % Sample parameters
            minv = rand(N, 1) * 2;
            d = 0.5 + rand(N, 1) * 0.5;
            K = 1 + rand(N, N) * 0.5;
            K = (K + K') / 2; % Do we need to symmetrize?
            for i=1:N
                K(i,i) = A(i,:) * K(:, i);
            end

            % Construct dynamic matrices
            A_dyn = zeros(2*N);
            B_dyn = zeros(2*N, N);
            for i=1:N
                for j=1:N
                    if i==j
                        A_dyn(2*i-1:2*i, 2*i-1:2*i) = ...
                            [1 dt; -K(i,i)*minv(i)*dt 1-d(i)*minv(i)*dt];
                        B_dyn(2*i-1:2*i, i) = [0; 1];
                    elseif A(i,j) == 1
                        A_dyn(2*i-1:2*i, 2*j-1:2*j) = [0 0; K(i,j)*minv(i)*dt 0];
                    end
                end
            end
        end
        function [Nx, Nu, p, q, A_dyn, B_dyn, Q, S, A] = grid(N, prob)
            Nx = 2*N;
            Nu = N;
            Q = eye(Nx);
            S = eye(Nu);
            p = 2;
            q = 1;
            
            n = sqrt(N);
            A = zeros(N);
            dt = 0.2;

            % Sample an adjacency matrix for the graph
            for i=1:N    
                % connect up
                if mod(i, n) ~= 0
                    if rand() < prob
                    A(i, i+1) = 1;
                    A(i+1, i) = 1;
                    end
                end
                % connect right
                if i + n <= N
                    if rand() < prob
                    A(i, i+n) = 1;
                    A(i+n, i) = 1;
                    end
                end
            end

            % Sample parameters
            minv = rand(N, 1) * 2;
            d = 0.5 + rand(N, 1) * 0.5;
            K = 1 + rand(N, N) * 0.5;
            K = (K + K') / 2; % Do we need to symmetrize?
            for i=1:N
                K(i,i) = A(i,:) * K(:, i);
            end

            % Construct dynamic matrices
            A_dyn = zeros(2*N);
            B_dyn = zeros(2*N, N);
            for i=1:N
                for j=1:N
                    if i==j
                        A_dyn(2*i-1:2*i, 2*i-1:2*i) = ...
                            [1 dt; -K(i,i)*minv(i)*dt 1-d(i)*minv(i)*dt];
                        B_dyn(2*i-1:2*i, i) = [0; 1];
                    elseif A(i,j) == 1
                        A_dyn(2*i-1:2*i, 2*j-1:2*j) = [0 0; K(i,j)*minv(i)*dt 0];
                    end
                end
            end
        end
        function [] = plot_grid(A)            % Plot the grid
            figure;
            N = size(A, 1);
            n = sqrt(N);
            for i=1:N
                for j=i+1:N
                    if A(i,j) == 1
                        % Plot an edge
                        pos_i_x = mod(i-1, n);
                        pos_i_y = -floor((i-1) / n);
                        pos_j_x = mod(j-1, n);
                        pos_j_y = -floor((j-1) / n);
                        plot([pos_i_x, pos_j_x], [pos_i_y, pos_j_y], 'b');
                        hold on
                    end
                end
            end
            xlim([-0.5, n-0.5]);
            ylim([-n+0.5, 0.5]);
        end
    end
end
