clear; clc; clf;
env_name = 'chain';
fn = strcat('../results/scalability_', env_name);
load(fn);

%% Process Chain Data 
for num = 1:length(networks)
    avg(num) = mean(times(num,:));
    stnd(num) = std(times(num,:));
    
    lower_curve(num) = avg(num)-stnd(num);
    upper_curve(num) = avg(num)+stnd(num);
end

color_chain = [0.4660 0.6740 0.1880];

%% Plot chain
figure(1)

semilogy(networks, avg, '-s', 'LineWidth', 4, 'MarkerSize', 12, ...
     'Color', color_chain, 'MarkerFaceColor', color_chain);
hold on

h = fill([networks,fliplr(networks)], [lower_curve,fliplr(upper_curve)],color_chain);
alpha(.4)
set(h,'edgecolor','white')

semilogy(networks, avg, '-s', 'LineWidth', 4, 'MarkerSize', 12, ...
     'Color', color_chain, 'MarkerFaceColor', color_chain, 'DisplayName',...
     'Chain');

%% Process Grid Data
clear;
env_name = 'grid';
fn = strcat('../results/scalability_', env_name);
load(fn);

for num = 1:length(networks)
    avg(num) = mean(times(num,:));
    stnd(num) = std(times(num,:));
    
    lower_curve(num) = avg(num)-stnd(num);
    upper_curve(num) = avg(num)+stnd(num);
end

color_grid = [0.8500 0.3250 0.0980];
%% Plot Grid
figure(1)

semilogy(networks, avg, '-s', 'LineWidth', 4, 'MarkerSize', 12, ...
     'Color', color_grid, 'MarkerFaceColor', color_grid);
hold on

h = fill([networks,fliplr(networks)], [lower_curve,fliplr(upper_curve)],color_grid);
alpha(.4)
set(h,'edgecolor','white')

semilogy(networks, avg, '-s', 'LineWidth', 4, 'MarkerSize', 12, ...
     'Color', color_grid, 'MarkerFaceColor', color_grid, 'DisplayName', ...
     'Grid');

xlabel('Network Size','interpreter','latex','Fontsize', 18)
ylabel('Time(s)','Interpreter','Latex','Fontsize', 18)
%ylim([0.03 0.1]);

title('\textbf{Runtime vs. network size}','Interpreter','Latex','Fontsize', 20)
leg1 = legend('','','Chain','','','Grid','Interpreter','Latex'); set(leg1, 'Fontsize', 18)
%% Save the figure
ax = gca;
exportgraphics(ax, strcat('~/Desktop/scalability_',env_name, '.png'));