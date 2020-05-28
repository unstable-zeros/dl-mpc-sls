%% Ploting code

fontsize1 = 40;
fontsize2 = 18;
fontsize3 = 30;
fontsize4 = 24;

LineWidth = 4;
MarkerSize = LineWidth*3;

scenario = {'scenario0.mat', 'scenario1.mat', 'scenario2.mat', 'scenario3.mat'};

color{1} = [37/255  154/255 197/255];% Blue
color{2} = [173/255 55/255  195/255];% Purple
color{3} = [202/255 100/255 60/255];% Orange
color{4} = [81/255  200/255 74/255];% Green

color{5} = [27/255  63/255 110/255];% Blue
color{6} = [65/255  28/255  78/255];% Purple
color{7} = [127/255 83/255 39/255];% Orange
color{8} = [41/255  110/255 37/255];% Green

figure (1)

for II = 1:max(size(scenario))

    load (['scenario' num2str(II-1) '.mat']) 

if II == 1
    subplot(2,2,II)
    plot(1:Tsim+1,x_VAL(1,:),'LineWidth',2,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x_VAL(3,:),'LineWidth',2,'Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    
     set(gca,'FontSize',fontsize4)
    
    xlabel('$$Time$$','interpreter','latex','Fontsize', fontsize1)
    ylabel('$$\theta_{1},\ \theta_{2}$$','Interpreter','Latex','Fontsize', fontsize1)
    
    xlim([0 50])
    ylim([-1 1.2])
    
    leg1 = legend('$$\theta_{1}$$', '$$\theta_{2}$$');
    set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize2)
    
    title(['Open loop dynamics'],'Fontsize', fontsize3)
else   
    subplot(2,2,II)
    plot(1:Tsim+1,x(1,:),'*','Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x_VAL(1,:),'LineWidth',2,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x(3,:),'*','Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    hold on
    plot(1:Tsim+1,x_VAL(3,:),'LineWidth',2,'Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    
     set(gca,'FontSize',fontsize4)
    
    xlabel('$$Time$$','interpreter','latex','Fontsize', fontsize1)
    ylabel('$$\theta_{1},\ \theta_{2}$$','Interpreter','Latex','Fontsize', fontsize1)
    
    xlim([0 50])
    ylim([-1 1.2])
    
    if II == 2
        leg1 = legend('$$\theta_{1}\ Centralized\ MPC$$', '$$\theta_{1}\ Localized\ MPC\ (Alg. I)$$','$$\theta_{2}\ Centralized\ MPC$$', '$$\theta_{2}\ Localized\ MPC\ (Alg. I)$$');
        set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize2)
    else 
        leg1 = legend('$$\theta_{1}\ Centralized\ MPC$$', '$$\theta_{1}\ Localized\ MPC\ (Alg. II)$$','$$\theta_{2}\ Centralized\ MPC$$', '$$\theta_{2}\ Localized\ MPC\ (Alg. II)$$');
        set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize2)
    end
    
    title(['Scenario' num2str(II-1)],'Fontsize', fontsize3)
%     
end
    
end

% sgtitle('Subsystems 1 and 2')
% set(leg,'Interpreter','latex','Fontsize', 12);



% figure (2)
% 
% for II = 1:max(size(scenario))
%     
%     load (['scenario' num2str(II) '.mat']) 
%     
%     plot(cases,iter,'-s','LineWidth',2,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
%     hold on
%     
% end
% 
% title('Average number of ADMM iterations with the number of states','Fontsize', 16)
% xlabel('$$Number\ of\ pendulums\ in\ the\ network$$','Interpreter','latex','Fontsize', 16)
% ylabel('$$Average\ number\ of\ ADMM\ iterations\ per\ MPC\ iteration\ for\ each\ state$$','Interpreter','latex','Fontsize', 16)
% leg = legend('$$Scenario\ 1:\ Algorithm\ 1$$',...
%               '$$Scenario\ 2:\ Algorithm\ 1$$',...
%               '$$Scenario\ 3:\ Algorithm\ 2$$', ...
%               '$$Scenario\ 4:\ Algorithm\ 2$$', '$$Scenario\ 4:\ Centralized\ MPC$$');
% set(leg,'Interpreter','latex','Fontsize', 12);


