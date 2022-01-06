%% Ploting code

scenario = {'Case1.mat', 'Case2.mat', 'Case3.mat', 'Case4.mat'};

color{1} = [37/255  154/255 197/255];% Blue
color{2} = [173/255 55/255  195/255];% Purple
color{3} = [202/255 100/255 60/255];% Orange
color{4} = [81/255  200/255 74/255];% Green

fontsize1 = 36;
fontsize2 = 28;
fontsize3 = 24;

LineWidth = 6;
MarkerSize = LineWidth*2;

figure (1)

for II = 1:max(size(scenario))

    load (['Case' num2str(II)]) 

    %subplot(1,2,1)
    plot(cases,time,'-s','LineWidth',LineWidth,'MarkerSize',MarkerSize,'Color',color{II},'MarkerEdge',color{II},'MarkerFaceColor',color{II})
    hold on
    
end

set(gca,'FontSize',fontsize3)
title('Runtime with the size of the network','Fontsize', fontsize2)
xlabel({'$$Number\ of\ subsystems\ in$$'; '$$the\ network$$'},'Interpreter','latex','Fontsize', fontsize1)
ylabel({'$$Average\ runtime\ per\ MPC\ iteration$$';'$$for\ each\ state\ (seconds)$$'},'Interpreter','latex','Fontsize', fontsize1)
leg = legend('$$Case\ 1:\ Algorithm\ 1$$',...
              '$$Case\ 2:\ Algorithm\ 1$$',...
              '$$Case\ 3:\ Algorithm\ 2$$',...
              '$$Case\ 4:\ Algorithm\ 2$$');...
set(leg,'Interpreter','latex','Fontsize', fontsize2);
xlim([min(cases) max(cases)]);
ylim([0,7])
