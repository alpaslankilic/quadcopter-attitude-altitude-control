function plot_results(t,x,u,input,p,visible)
%PLOT_RESULTS Position, cascaded attitude control, and actuator/gust evidence.
blue=[0 .36 .60];names={'World x [m]','World y [m]','Altitude [m]'};
[fig,layout]=new_figure('Position tracking',3,visible);
for axis=1:3
    ax=nexttile(layout);hold(ax,'on');
    plot(ax,t,input(:,axis),':','Color',[.45 .45 .45],'LineWidth',1.3,'DisplayName','Position request');
    plot(ax,t,x(:,axis),'Color',blue,'LineWidth',1.7,'DisplayName','True position');
    events(ax,p);ylabel(ax,names{axis});style(ax);
    if axis==1,legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');end
end
xlabel(ax,'Time [s]');save_figure(fig,'01_position_tracking',visible);
[fig,layout]=new_figure('Attitude regulation',3,visible);names={'Roll [deg]','Pitch [deg]','Yaw [deg]'};
for axis=1:3
    ax=nexttile(layout);hold(ax,'on');
    plot(ax,t,rad2deg(u(:,8+axis)),'--','Color',[.45 .45 .45],'LineWidth',1.2,'DisplayName','Attitude request');
    plot(ax,t,rad2deg(x(:,6+axis)),'Color',blue,'LineWidth',1.6,'DisplayName','True attitude');
    events(ax,p);ylabel(ax,names{axis});style(ax);
    if axis==1,legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');end
end
xlabel(ax,'Time [s]');save_figure(fig,'02_attitude_control',visible);
[fig,layout]=new_figure('Rotors and external gust',3,visible);
ax=nexttile(layout);plot(ax,t,x(:,13:16),'LineWidth',1.3);events(ax,p);ylabel(ax,'Rotor speed [rad/s]');style(ax);
legend(ax,'Rotor 1','Rotor 2','Rotor 3','Rotor 4','Location','southoutside','Orientation','horizontal','Box','off');
ax=nexttile(layout);plot(ax,t,input(:,11:13),'LineWidth',1.4);events(ax,p);ylabel(ax,'External force [N]');style(ax);
legend(ax,'World x','World y','World z','Location','southoutside','Orientation','horizontal','Box','off');
ax=nexttile(layout);plot(ax,t,input(:,14:16),'LineWidth',1.4);events(ax,p);ylabel(ax,'External moment [N m]');xlabel(ax,'Time [s]');style(ax);
legend(ax,'Body x','Body y','Body z','Location','southoutside','Orientation','horizontal','Box','off');
save_figure(fig,'03_rotors_and_gust',visible);
end

function events(ax,p)
xlim(ax,[0,p.stopTime]);xline(ax,p.moveTime,':','Move begins','Color',[.5 .5 .5], ...
    'LabelOrientation','horizontal','FontSize',9,'HandleVisibility','off');
xline(ax,p.gustTime,'--','Gust applied','Color',[.45 .2 .55], ...
    'LabelOrientation','horizontal','FontSize',10,'HandleVisibility','off');
xline(ax,p.gustTime+p.gustDuration,':','Color',[.45 .2 .55],'HandleVisibility','off');
limits=ylim(ax);ylim(ax,[limits(1),limits(2)+.18*diff(limits)]);
end

function [fig,layout]=new_figure(name,panels,visible)
state='off';if visible,state='on';end
fig=figure('Name',name,'Visible',state,'Color','w','Position',[100 60 980 440+160*(panels-1)]);
set(fig,'DefaultAxesColorOrder',[0 .36 .60;.8 .32 .05;.2 .55 .35;.5 .3 .65]);
layout=tiledlayout(fig,panels,1,'TileSpacing','compact','Padding','loose');
end

function style(ax)
set(ax,'FontName','Arial','FontSize',12,'Box','off','TickDir','out', ...
    'YGrid','on','GridAlpha',.12,'LineWidth',.8);
end

function save_figure(fig,name,visible)
exportgraphics(fig,fullfile('results',[name '.png']),'Resolution',200,'Padding',8,'Units','points');
exportgraphics(fig,fullfile('results',[name '.pdf']),'ContentType','vector','Padding',8,'Units','points');
if ~visible,close(fig);end
end
