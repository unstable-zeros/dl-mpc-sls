%% Trajectory Length vs. Network Size
env_name = 'chain';
load(strcat('../results/tl_net_', env_name));

color_distributed = [0.4660 0.6740 0.1880];
color_centralized = [0.8500 0.3250 0.0980];

% Plot the centralized trajectory length growth
semilogy((nn_min:nn_max).^2, centralized_tl, '-s', 'LineWidth', 4, ...
        'MarkerSize', 12, 'Color', color_centralized, 'MarkerFaceColor', ...
        color_centralized);
hold on

% Plot the distributed traj length growth
ddd_tl_array = zeros(length(ddd_tl), 1);
for i=1:length(ddd_tl)
    ddd_tl_array(i) = mean(ddd_tl{i});
end
semilogy((nn_min:nn_max).^2, ddd_tl_array, '-s', 'LineWidth', 4, ...
        'MarkerSize', 12, 'Color', color_distributed, 'MarkerFaceColor', ...
        color_distributed);

xlabel('Network Size','interpreter','latex','Fontsize', 18)
ylabel('Past trajectory length','Interpreter','Latex','Fontsize', 18)

leg1 = legend('Centralized','Distributed');
set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', 15);
set(leg1,'Location','northwest');

title('\textbf{Amount of data vs. network size}','Interpreter','Latex',...
    'Fontsize', 20)

%% Save the figure
ax = gcf;
exportgraphics(ax, strcat('~/Desktop/tl_network_', env_name, '.png'));