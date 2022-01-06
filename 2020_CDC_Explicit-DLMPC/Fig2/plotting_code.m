%% Ploting code

scenario = {'Case1', 'Case2.mat', 'Case3.mat'};

color{1} = [37/255  154/255 197/255];
color{2} = [27/255, 27/255, 171/255];
color{3} = [173/255 55/255  195/255];


fontsize1 = 40;
fontsize2 = 40;

figure (1)

for II = 1:max(size(scenario))
    
    if II ~= 3

    load (scenario{II}) 

    plot(cases,time,':s','LineWidth',2,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    
    else
        
    load (scenario{II}) 
    
    plot(cases,time,'-s','LineWidth',2,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
        
    end
    
end

title('Runtime with the size of the network','Fontsize', fontsize1)
xlabel({'Number\ of\ subsystems\ in\ the\ network'},'Interpreter','latex','Fontsize', fontsize1)
ylabel({'Average\ runtime\ per\ MPC';'iteration\ for\ each\ state'; '(seconds)'},'Interpreter','latex','Fontsize', fontsize1)
leg = legend('Case\ 1:\ closed\ form',...
             'Case\ 2:\ online\ optimization',...
             'Case\ 3:\ explicit\ solution');...
set(leg,'Interpreter','latex','Fontsize', fontsize2);
xlim([min(cases) max(cases)]);
ylim([0,4])

hold off
