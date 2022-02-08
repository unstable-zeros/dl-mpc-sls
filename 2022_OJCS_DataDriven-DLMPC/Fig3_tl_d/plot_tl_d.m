%% Trajectory Length vs. Locality Size

env_name = 'chain';
load(strcat('../results/tl_d_', env_name));
color_distributed = [0.4660 0.6740 0.1880];

plot((d_min:d_max), mean(d_tl, 2), '-s', 'LineWidth', 4, ...
        'MarkerSize', 12, 'Color', color_distributed, 'MarkerFaceColor', ...
        color_distributed);
xlabel('Locality $$d$$','interpreter','latex','Fontsize', 18)
ylabel('Trajectory Length','Interpreter','Latex','Fontsize', 18)
title('\textbf{Amount of data vs. Locality}','Interpreter','Latex',...
    'Fontsize', 20)
%% Save the figure
ax = gcf;
exportgraphics(ax, strcat('~/Desktop/tl_d_', env_name, '.png'));

