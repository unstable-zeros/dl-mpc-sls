env_name = 'chain';
fn = strcat('../results/scalability_', env_name);
load(fn);

%% Process Data 
for num = 1:length(networks)
    avg(num) = mean(times(num,:));
    stnd(num) = std(times(num,:));
    
    lower_curve(num) = avg(num)-stnd(num);
    upper_curve(num) = avg(num)+stnd(num);
end

color = [0.4660 0.6740 0.1880];

%% Plot
figure(1)


semilogy(networks, avg, '-s', 'LineWidth', 4, 'MarkerSize', 12, ...
     'Color', color, 'MarkerFaceColor', color);
hold on

h = fill([networks,fliplr(networks)], [lower_curve,fliplr(upper_curve)],color);
alpha(.4)
set(h,'edgecolor','white')

semilogy(networks, avg, '-s', 'LineWidth', 4, 'MarkerSize', 12, ...
     'Color', color, 'MarkerFaceColor', color);

xlabel('Network Size','interpreter','latex','Fontsize', 18)
ylabel('Time(s)','Interpreter','Latex','Fontsize', 18)
ylim([0.03 0.1]);

title('\textbf{Runtime vs. network size}','Interpreter','Latex','Fontsize', 20)

%% Save the figure
ax = gca;
exportgraphics(ax, strcat('~/Desktop/scalability_',env_name, '.png'));