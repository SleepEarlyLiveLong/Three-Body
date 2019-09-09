%% 数据准备
% 数据读入
load([pwd,'\data\EPAP5.mat'])

% 方便起见，改变变量名
E = Param.E(2:min(1001,end),:);
P = Param.P(2:min(1001,end),:);
AP = Param.AP(2:min(1001,end),:);
R12 = Param.R12(2:min(1001,end),:);
R13 = Param.R13(2:min(1001,end),:);
R23 = Param.R23(2:min(1001,end),:);

%% 绘图-能量
figure;

ax1 = subplot(2,3,1);
plot(E(:,3),'color','r');
xlabel('时间/天');ylabel('系统总能量/焦耳');title('系统总能量随时间的变化');
% legend(ax1,'calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
deltaE = E(:,3)-E(1,3);
deltaErate = deltaE/abs(E(1,3))*100;
plot(deltaErate,'color','r');
xlabel('时间/天');ylabel('deltaE/E0 /%');title({'系统总能量变动比例';'随时间的变化'});

subplot(2,3,2);
plot(E(:,1),'color','r');
xlabel('时间/天');ylabel('系统总动能/焦耳');title('系统总动能随时间的变化');

subplot(2,3,5);
deltaE = E(:,1)-E(1,1);
deltaErate = deltaE/abs(E(1,1))*100;
plot(deltaErate,'color','r');
xlabel('时间/天');ylabel('deltaEk/Ek0 /%');title({'系统总动能变动比例';'随时间的变化'});

subplot(2,3,3);
plot(E(:,2),'color','r');
xlabel('时间/天');ylabel('系统总引力势能/焦耳');title({'系统总引力势能';'随时间的变化'});

subplot(2,3,6);
deltaE = E(:,2)-E(1,2);
deltaErate = deltaE/abs(E(1,2))*100;
plot(deltaErate,'color','r');
xlabel('时间/天');ylabel('deltaEg/Eg0 /%');title({'系统总引力势能变动比例';'随时间的变化'});

%% 绘图-动量
figure;

subplot(2,3,1);
plot3(P(:,1),P(:,2),P(:,3),'color','r');grid on;hold on         % 轨迹
scatter3(P(1,1),P(1,2),P(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r');    % 起点
scatter3(P(end,1),P(end,2),P(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','r');  % 终点
% 补充
xlabel('X轴/ kg*m/s');ylabel('Y轴/ kg*m/s');zlabel('Z轴/ kg*m/s');title('动量随时间的变化轨迹');

subplot(2,3,2);
plot(P(:,4),'color','r');
xlabel('时间/天');ylabel('系统总动量/ kg*m/s');title({'系统总动量';'随时间的变化'});
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,3);
deltaPrate = (P(:,5)-P(1,5))/P(1,5)*100;
plot(deltaPrate,'color','r');
xlabel('时间/天');ylabel('deltaABSP/ABSP0 /%');title({'系统总绝对动量误差';'随时间的变化'});

subplot(2,3,4);
plot(P(:,1),'color','r');
xlabel('时间/天');ylabel('系统X轴动量/ kg*m/s');title({'系统X轴动量';'随时间的变化'});

subplot(2,3,5);
plot(P(:,2),'color','r');
xlabel('时间/天');ylabel('系统Y轴动量/ kg*m/s');title({'系统Y轴动量';'随时间的变化'});

subplot(2,3,6);
plot(P(:,3),'color','r');
xlabel('时间/天');ylabel('系统Z轴动量/ kg*m/s');title({'系统Z轴动量';'随时间的变化'});

%% 绘图-角动量
figure;

subplot(2,2,1);
plot3(AP(:,1),AP(:,2),AP(:,3),'color','r');grid on;hold on;
scatter3(AP(1,1),AP(1,2),AP(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r');
scatter3(AP(end,1),AP(end,2),AP(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','r');
legend('trace','start point','stop point');
% 补充
xlabel('X轴/ kg*m^2/s');ylabel('Y轴/ kg*m^2/s');zlabel('Z轴/ kg*m^2/s');title('角动量随时间的变化轨迹');

subplot(2,2,2);
plot(AP(:,4),'color','r');
xlabel('时间/天');ylabel('系统总角动量/ kg*m^2/s');title({'系统总角动量';'随时间的变化'});

subplot(2,4,5);
plot(AP(:,1),'color','r');
xlabel('时间/天');ylabel('系统X轴角动量/ kg*m^2/s');title({'系统X轴角动量';'随时间的变化'});

subplot(2,4,6);
plot(AP(:,2),'color','r');
xlabel('时间/天');ylabel('系统Y轴角动量/ kg*m^2/s');title({'系统Y轴角动量';'随时间的变化'});

subplot(2,4,7);
plot(AP(:,3),'color','r');
xlabel('时间/天');ylabel('系统Z轴角动量/ kg*m^2/s');title({'系统Z轴角动量';'随时间的变化'});

subplot(2,4,8);
deltaAP = AP(:,4)-AP(1,4);
deltaAPrate = deltaAP/abs(AP(1,4))*100;
plot(deltaAPrate,'color','r');
xlabel('时间/天');ylabel('deltaAP/AP0 /%');title({'系统总角动量误差';'随时间的变化'});

%% 绘图-质点间距
figure;

subplot(2,3,1);
plot(R12,'color','r');
xlabel('时间/天');ylabel('间距/米');title('两星间距R12随时间的变化');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
R12rate = (R12-R12(1))/R12(1)*100;
plot(R12rate,'color','r');
xlabel('时间/天');ylabel('间距变化量/%');title('两星间距R12的变化量随时间的变化');

subplot(2,3,2);
plot(R13,'color','r');
xlabel('时间/天');ylabel('间距/米');title('两星间距R13随时间的变化');

subplot(2,3,5);
R13rate = (R13-R13(1))/R13(1)*100;
plot(R13rate,'color','r');
xlabel('时间/天');ylabel('间距变化量/%');title('两星间距R13的变化量随时间的变化');

subplot(2,3,3);
plot(R23,'color','r');
xlabel('时间/天');ylabel('间距/米');title('两星间距R23随时间的变化');

subplot(2,3,6);
R23rate = (R23-R23(1))/R23(1)*100;
plot(R23rate,'color','r');
xlabel('时间/天');ylabel('间距变化量/%');title('两星间距R23的变化量随时间的变化');
