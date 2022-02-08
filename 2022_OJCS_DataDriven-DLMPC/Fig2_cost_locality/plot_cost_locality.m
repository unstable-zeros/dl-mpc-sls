env_name = 'chain';
load(strcat('../results/cost_locality_', env_name));

%% Plotting
color_distributed = [0.4660 0.6740 0.1880];
color_cent = [0 0.1059 0.6392];

figure(1)
plot(1:5, costs, '-s', 'LineWidth', 4, ...
    'MarkerSize', 12, 'Color', color_distributed, 'MarkerFaceColor', ...
    color_distributed);
hold on
plot(1:5, ones(5, 1)* obj_cent, '--', 'LineWidth', 4, 'Color', color_cent);
xlabel('Locality $$d$$','interpreter','latex','Fontsize', 18)
ylabel('Cost','Interpreter','Latex','Fontsize', 18)

leg1 = legend('DLMPC','Cent. MPC');
set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', 18)

title('\textbf{Cost vs. Locality}','Interpreter','Latex',...
    'Fontsize', 20)

%% Save the figure
ax = gca;
exportgraphics(ax, strcat('~/Desktop/cost_locality_', env_name, '.png'));