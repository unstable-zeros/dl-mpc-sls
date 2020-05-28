%% Ploting code

%% WITH SCENARIO 0 (CDC PAPER)

fontsize1 = 40;
fontsize2 = 20;
fontsize3 = 32;
fontsize4 = 24;

LineWidth = 4;
MarkerSize = LineWidth*3;

scenario = {'scenario0.mat', 'scenario1_no_tc.mat', 'scenario1_tc.mat'};

color{1} = [37/255  154/255 197/255];% Blue
color{2} = [173/255 55/255  195/255];% Purple
color{3} = [81/255  200/255 74/255];% Green

color{5} = [27/255  63/255 110/255];% Blue
color{6} = [65/255  28/255  78/255];% Purple
color{7} = [41/255  110/255 37/255];% Green

figure (1)

for II = 1:max(size(scenario))

    load (scenario{II}) 

if II == 1
    subplot(2,3,II)
    plot(1:Tsim+1,x_VAL(1,:),'LineWidth',LineWidth,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x_VAL(3,:),'LineWidth',LineWidth,'Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    
    set(gca,'FontSize',fontsize4)
    
    xlabel('$$Time$$','interpreter','latex','Fontsize', fontsize1)
    ylabel('$$\theta_{1},\ \theta_{2}$$','Interpreter','Latex','Fontsize', fontsize1)
    
    xlim([0 50])
    ylim([-1 1.2])
    
    leg1 = legend('$$\theta_{1}$$', '$$\theta_{2}$$');
    set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize2)
    
%     title(['Open loop dynamics'],'Fontsize', 16)
else   
    subplot(2,3,II)
    plot(1:Tsim+1,x(1,:),'*','MarkerSize',MarkerSize,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x_VAL(1,:),'LineWidth',LineWidth,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x(3,:),'*','MarkerSize',MarkerSize,'Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    hold on
    plot(1:Tsim+1,x_VAL(3,:),'LineWidth',LineWidth,'Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    
    set(gca,'FontSize',fontsize4)
    
    xlabel('$$Time$$','interpreter','latex','Fontsize', fontsize1)
    ylabel('$$\theta_{1},\ \theta_{2}$$','Interpreter','Latex','Fontsize', fontsize1)
    
    xlim([0 50])
    ylim([-1 1.2])
    
    leg1 = legend('$$\theta_{1}\ Centralized\ MPC$$', '$$\theta_{1}\ DLMPC\ (Alg. I)$$','$$\theta_{2}\ Centralized\ MPC$$', '$$\theta_{2}\ DLMPC\ (Alg. I)$$');
    set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize2)
    
%     title(['Closed loop dynamics (MPC)'],'Fontsize', 16)
    
end
    
end

% sgtitle('Subsystems 1 and 2')
% set(leg,'Interpreter','latex','Fontsize', 12);



% figure (2)

for II = 2:max(size(scenario))
    
    load (scenario{II}) 
    
    if II == 2
        t0 = 3;
    else
        t0 = 4;
    end
    
    set(gca,'FontSize',fontsize4)
    
    subplot(2,1,2)
    plot(t0:10,obj,'-','LineWidth',LineWidth,'Color',color{II})%'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(t0:10,obj_VAL,'*','MarkerSize',MarkerSize,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on  
    
    xlabel('$$Time\ horizon$$','interpreter','latex','Fontsize', fontsize1)
    ylabel('$$Cost$$','Interpreter','Latex','Fontsize', fontsize1)
    
end

    leg1 = legend('$$Without\ terminal\ constraint\ (DLMPC)$$', '$$Without\ terminal\ constraint\ (Centralized\ MPC)$$', '$$With\ terminal\ constraint\ x_T=0\ (DLMPC)$$', '$$With\ terminal\ constraint\ x_T=0\ (Centralized\ MPC)$$');
    set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize3)

% title('Average number of ADMM iterations with the number of states','Fontsize', 16)
% xlabel('$$Number\ of\ pendulums\ in\ the\ network$$','Interpreter','latex','Fontsize', 16)
% ylabel('$$Average\ number\ of\ ADMM\ iterations\ per\ MPC\ iteration\ for\ each\ state$$','Interpreter','latex','Fontsize', 16)
% leg = legend('$$Scenario\ 1:\ Algorithm\ 1$$',...
%               '$$Scenario\ 2:\ Algorithm\ 1$$',...
%               '$$Scenario\ 3:\ Algorithm\ 2$$', ...
%               '$$Scenario\ 4:\ Algorithm\ 2$$', '$$Scenario\ 4:\ Centralized\ MPC$$');
% set(leg,'Interpreter','latex','Fontsize', 12);


%% WITHOUTH SCENARIO 0 (ARXIV PAPER)

%% Ploting code

fontsize1 = 40;
fontsize2 = 18;
fontsize3 = 30;
fontsize4 = 24;

LineWidth = 4;
MarkerSize = LineWidth*3;

scenario = {'scenario1_no_tc.mat', 'scenario1_tc.mat'};

color{1} = [173/255 55/255  195/255];% Purple
color{2} = [81/255  200/255 74/255];% Green

color{5} = [65/255  28/255  78/255];% Purple
color{6} = [41/255  110/255 37/255];% Green

figure (1)

for II = 1:max(size(scenario))

    load (scenario{II}) 
  
    subplot(2,2,II)
    plot(1:Tsim+1,x(1,:),'*','MarkerSize',MarkerSize,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x_VAL(1,:),'LineWidth',LineWidth,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(1:Tsim+1,x(3,:),'*','MarkerSize',MarkerSize,'Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    hold on
    plot(1:Tsim+1,x_VAL(3,:),'LineWidth',LineWidth,'Color',color{II+4},'MarkerEdge',color{II+4},'MarkerFaceColor',color{II+4})
    
    set(gca,'FontSize',fontsize4)
    
    xlabel('$$Time$$','interpreter','latex','Fontsize', fontsize1)
    ylabel('$$\theta_{1},\ \theta_{2}$$','Interpreter','Latex','Fontsize', fontsize1)
    
    xlim([0 50])
    ylim([-1 1.2])
    
    leg1 = legend('$$\theta_{1}\ Centralized\ MPC$$', '$$\theta_{1}\ DLMPC\ (Alg. I)$$','$$\theta_{2}\ Centralized\ MPC$$', '$$\theta_{2}\ DLMPC\ (Alg. I)$$');
    set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize2)
    
%     title(['Closed loop dynamics (MPC)'],'Fontsize', 16)
    
end

% sgtitle('Subsystems 1 and 2')
% set(leg,'Interpreter','latex','Fontsize', 12);



% figure (2)

for II = 1:max(size(scenario))
    
    load (scenario{II}) 
    
    if II == 2
        t0 = 3;
    else
        t0 = 4;
    end
    
    set(gca,'FontSize',fontsize4)
    
    subplot(2,1,2)
    plot(t0:10,obj,'-','LineWidth',LineWidth,'Color',color{II})%'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    plot(t0:10,obj_VAL,'*','MarkerSize',MarkerSize,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on  
    
    xlabel('$$Time\ horizon$$','interpreter','latex','Fontsize', fontsize1)
    ylabel('$$Cost$$','Interpreter','Latex','Fontsize', fontsize1)
    
end

    leg1 = legend('$$Without\ terminal\ constraint\ (DLMPC)$$', '$$Without\ terminal\ constraint\ (Centralized\ MPC)$$', '$$With\ terminal\ constraint\ x_T=0\ (DLMPC)$$', '$$With\ terminal\ constraint\ x_T=0\ (Centralized\ MPC)$$');
    set(leg1,'Interpreter','latex'); set(leg1, 'Fontsize', fontsize3)

% title('Average number of ADMM iterations with the number of states','Fontsize', 16)
% xlabel('$$Number\ of\ pendulums\ in\ the\ network$$','Interpreter','latex','Fontsize', 16)
% ylabel('$$Average\ number\ of\ ADMM\ iterations\ per\ MPC\ iteration\ for\ each\ state$$','Interpreter','latex','Fontsize', 16)
% leg = legend('$$Scenario\ 1:\ Algorithm\ 1$$',...
%               '$$Scenario\ 2:\ Algorithm\ 1$$',...
%               '$$Scenario\ 3:\ Algorithm\ 2$$', ...
%               '$$Scenario\ 4:\ Algorithm\ 2$$', '$$Scenario\ 4:\ Centralized\ MPC$$');
% set(leg,'Interpreter','latex','Fontsize', 12);