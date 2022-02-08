env_name = 'chain';
load(strcat('../results/optimality_', env_name));

color_distributed = [0.4660 0.6740 0.1880];
color_centralized = [0.8500 0.3250 0.0980];

%% Plot
figure(1)

% Plot dlmpc trajectory
subplot(1,2,1);
plot(1:Tsim+1,x_cent(1,:),'-', 'LineWidth', 4, ...
    'MarkerSize', 6, 'Color', color_centralized, 'MarkerFaceColor', ...
    color_centralized)
hold on
plot(1:Tsim+1,x_ddd(1,:), 's', 'LineWidth', 4, ...
    'MarkerSize', 6, 'Color', color_distributed, 'MarkerFaceColor', ...
    color_distributed)

xlabel('Time','interpreter','latex','Fontsize', 18)
ylabel('$$\theta_{1}$$','Interpreter','Latex','Fontsize', 18)
leg1 = legend('Model-based','Alg. 1');
set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', 15)

subplot(1,2,2);
plot(1:Tsim+1,x_cent(2,:),'-', 'LineWidth', 4, ...
    'MarkerSize', 6, 'Color', color_centralized, 'MarkerFaceColor', ...
    color_centralized)
hold on
plot(1:Tsim+1,x_ddd(2,:), 's', 'LineWidth', 4, ...
    'MarkerSize', 6, 'Color', color_distributed, 'MarkerFaceColor', ...
    color_distributed)
xlabel('Time','interpreter','latex','Fontsize', 18)
ylabel('$$\omega_{1}$$','Interpreter','Latex','Fontsize', 18)
leg1 = legend('Model-based','Alg. 1');
set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', 15)


sgtitle('\textbf{Evolution of System Trajectory}','Interpreter','Latex',...
    'Fontsize', 20);
%% Save the figure
ax = gcf;
exportgraphics(ax, strcat('~/Desktop/optimality_', env_name, '.png'));