classdef    data_driven_lib 
    methods     ( Static = true )
        function [xtraj, utraj] = generate_traj(A, B, T)
            % Generates a trajectory
            % get parameters
            [Nx, Nu] = size(B);
            xtraj = zeros(Nx, T+1);
            utraj = zeros(Nu, T);

            % random initial condition
            xtraj(:, 1) = randn(Nx, 1);

            % simulate trajectory using (LQR controller + noise)
            [~, K, ~] = idare(A, B, eye(Nx), eye(Nu), [], []);
            for t = 1:T
                x = xtraj(:, t);
                u = -K * x + rand(Nu, 1) * 10;
                xtraj(:, t+1) = A * x + B * u;
                utraj(:, t) = u;
            end
        end
        
        function [H] = hankelize(signal, L)
            % Hankelize a matrix
            % signal    - the signal to be converted to Hankel matrix form
            % L         - the order of the Hankel matrix
            [s, T] = size(signal);   % s denotes the signal dimension
            H = zeros(L*s, T-L+1);
            for i=1:L
                H((i-1)*s+1:i*s, :) = signal(:, i:T-L+i);
            end
        end
        
        function [mask] = patch_mask(A, d, state_dim, sig_dim, i)
            % Picking out columns relevant to locality
            % signal_dim is the dimension of the desired signal; 
            % it could be the state, or it could be the control.
            A_subsys = data_driven_lib.subsystem_adjacency(A, state_dim);
            path_mat = A_subsys^d>0;
            patch_subsys = path_mat(:, i); % See which subsystems I affect
            mask = logical(kron(patch_subsys, ones(sig_dim, 1)));
            
            %comms_adj = abs(A)>0;
            %localityR = comms_adj^d>0;
            %subsystem_rows = (state_dim)*(i-1)+1 : state_dim * i;
            %mask = any(localityR(subsystem_rows, :), 1);
        end
        
        function [mask] = control_mask_from_state(p, q, state_mask)
            % Get control mask from state mask
            [~, Nx] = size(state_mask);
            N = Nx / p;
            mask = zeros(1, N*q);
            for i=1:N
                x_ind = p*(i-1)+1 : p*i;
                u_ind = q*(i-1)+1 : q*i;
                mask(u_ind) = any(state_mask(x_ind), 'all');
            end
            mask = logical(mask);
        end
        
        function [sys_adj] = subsystem_adjacency(A, state_dim)
            % Find subsystem adjacency matrix
            [Nx, ~] = size(A);
            num_systems = Nx / state_dim;
            sys_adj = zeros(num_systems, num_systems);
            for i=1:num_systems
                for j=1:num_systems
                    i_ind = (state_dim)*(i-1)+1 : state_dim * i;
                    j_ind = (state_dim)*(j-1)+1 : state_dim * j;
                    sys_adj(i,j) = any(A(i_ind,j_ind) ~= 0, 'all');
                end
            end
        end
        
        function [mask] = subsys_mask(joint_dim, subsys_dim, i)
            % Get subsystem mask
            int_mask = zeros(1, joint_dim);
            int_mask(:, (i-1)*(subsys_dim)+1:i*subsys_dim) = 1;            
            mask = logical(int_mask)';
        end
        
        function [Hx, Hu, Hb] = sep_subsystem_hankel(A, B, p, q, d, L)
            % Sample a hankel matrix for each patch separately
            % Preallocate cell array
            [Nx, Nu] = size(B);
            N = Nx / p;
            Hx = cell(N, 1);
            Hu = cell(N, 1);
            Hb = cell(N, 1);
            % construct local hankel matrices
            for i=1:N
                % global masks
                x_mask = data_driven_lib.patch_mask(A, d, p, p, i);
                xx_mask = data_driven_lib.patch_mask(A, d+2, p, p, i);
                x_bd_mask = xor(x_mask, xx_mask);
                u_mask = data_driven_lib.patch_mask(A, d+1, p, q, i);
                uu_mask = data_driven_lib.patch_mask(A, d+2, p, q, i);
                u_bd_mask = xor(u_mask, uu_mask);
                % masks within the patch
                x_sub_bd_mask = and(xx_mask, x_bd_mask);
                x_sub_bd_mask = x_sub_bd_mask(xx_mask == 1);
                x_sub_bd_mask = repmat(x_sub_bd_mask, L, 1);
                u_sub_bd_mask = and(uu_mask, u_bd_mask);
                u_sub_bd_mask = u_sub_bd_mask(uu_mask == 1);
                u_sub_bd_mask = repmat(u_sub_bd_mask, L, 1);
                % Collect subsystem trajectory
                Ai = A(xx_mask, xx_mask);
                Bi = B(xx_mask, uu_mask);
                Nx_patch = sum(xx_mask);
                Nu_patch = sum(uu_mask);
                Ti = (Nx_patch+L) * Nu_patch + Nx_patch + L - 1;
                [xtraj, utraj] = data_driven_lib.generate_traj(Ai, Bi, Ti);
                % Hankelize
                xhank = data_driven_lib.hankelize(xtraj(:, 1:Ti), L);
                uhank = data_driven_lib.hankelize(utraj, L);
                Hx{i} = xhank(~x_sub_bd_mask, :);
                Hu{i} = uhank(~u_sub_bd_mask, :);
                Hb{i} = [xhank(x_sub_bd_mask, :); uhank(u_sub_bd_mask, :)];
            end
        end
        
        function [Hx, Hu, Hb] = subsystem_hankel(A, B, p, q, d, L)
            % Take global hankel matrices and construct local ones
            % Preallocate cell array
            [Nx, Nu] = size(B);
            N = Nx / p;
            Hx = cell(N, 1);
            Hu = cell(N, 1);
            Hb = cell(N, 1);
            % Compute necessary trajectory length
            %Nx_patch = min((2*(d+1)+1)*p, Nx);
            %Nu_patch = min((2*(d+2)+1)*q, Nu);
            %traj_length = (Nx_patch+L) * Nu_patch + Nx_patch + L - 1;
            traj_lengths = data_driven_lib.find_traj_length(A, p, q, d, L);
            [xtraj, utraj] = data_driven_lib.generate_traj(A, B, ...
                max(traj_lengths));
            %xhank = data_driven_lib.hankelize(xtraj(:, 1:traj_length), L);
            %uhank = data_driven_lib.hankelize(utraj, L);
            % Construct local hankel matrices
            for i=1:N
                xhanki = data_driven_lib.hankelize(...
                    xtraj(:, 1:traj_lengths(i)), L);
                uhanki = data_driven_lib.hankelize(...
                    utraj(:, 1:traj_lengths(i)), L);
                % Compute masks
                x_mask = data_driven_lib.patch_mask(A, d, p, p, i);
                xx_mask = data_driven_lib.patch_mask(A, d+2, p, p, i);
                u_mask = data_driven_lib.patch_mask(A, d+1, p, q, i);
                uu_mask = data_driven_lib.patch_mask(A, d+2, p, q, i);
                x_bd = xor(x_mask, xx_mask);
                u_bd = xor(u_mask, uu_mask);
                % Construct the patch hankel matrices
                x_mask = repmat(x_mask, L, 1);
                u_mask = repmat(u_mask, L, 1);
                x_bd = repmat(x_bd, L, 1);
                u_bd = repmat(u_bd, L, 1);
                Hx{i} = xhanki(x_mask, :);
                Hu{i} = uhanki(u_mask, :);
                Hb{i} = [xhanki(x_bd, :); uhanki(u_bd, :)];
            end
        end
        
        function [shared_x_ind, shared_u_ind] = find_overlap(A, p, q, d, T)
            N = size(A, 1) / p;
            Phix_count = zeros(N*p, N*p);
            Phiu_count = zeros(N*q, N*p);
            for i=1:N
                x_patch_mask = data_driven_lib.patch_mask(A, d+1, p, p, i);
                u_patch_mask = data_driven_lib.patch_mask(A, d+2, p, q, i);
                %subsys_mask = data_driven_lib.subsys_mask(size(A, 1), p, i);
                Phi_x_mask = x_patch_mask * x_patch_mask';
                Phi_u_mask = u_patch_mask * x_patch_mask';
                Phix_count = Phix_count + Phi_x_mask;
                Phiu_count = Phiu_count + Phi_u_mask;
            end
            Phix_count = repmat(Phix_count, T, 1);
            Phiu_count = repmat(Phiu_count, T, 1);
            
            % Assign indices to all shared variables
            xmask = Phix_count > 1;      
            shared_x_ind = zeros(size(Phix_count));
            shared_x_ind(xmask) = 1:sum(xmask, 'all');
            umask = Phiu_count > 1; 
            shared_u_ind = zeros(size(Phiu_count));
            shared_u_ind(umask) = (1:sum(umask, 'all')) + sum(xmask, 'all');
        end
        
        function [traj_lengths] = find_traj_length(A, p, q, d, T)
            [Nx, ~] = size(A);
            N = Nx / p;
            traj_lengths = zeros(N, 1);
            for i=1:N
                x_mask = data_driven_lib.patch_mask(A, d+1, p, p, i);
                u_mask = data_driven_lib.patch_mask(A, d+2, p, q, i);
                u_bar_mask = xor(x_mask,...
                    data_driven_lib.patch_mask(A, d+2, p, p, i));
                nx = sum(x_mask);
                nu = sum(u_mask) + sum(u_bar_mask);
                traj_lengths(i) = (nx+T)* (nu+1) - 1;
            end
            traj_length = max(traj_lengths);
        end
        
        function [ZZ] = construct_Z(A, B, T)
            [Nx, ~] = size(A);
            ZA = kron(eye(T), A);
            ZA = [zeros(Nx, size(ZA, 2)); ZA(1:end-Nx, :)];
            ZB = kron(eye(T), B);
            ZB = [zeros(Nx, size(ZB, 2)); ZB(1:end-Nx, :)];
            ZZ = [eye(size(ZA, 1))-ZA -ZB];
        end
    end
end




















