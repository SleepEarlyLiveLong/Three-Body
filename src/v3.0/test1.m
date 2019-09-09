%% 数据准备
% 数据读入
load([pwd,'\data\EPAP1_min1.mat'])
Param_min = Param;
load([pwd,'\data\EPAP1_hour1.mat'])
Param_hour = Param;
load([pwd,'\data\EPAP1_day1.mat'])
Param_day = Param;
% 最短记录时长
% len = min([size(Param_min.E,1) size(Param_hour.E,1) size(Param_day.E,1)]);
len = 1001;
% 方便起见，改变变量名
E_mim = Param_min.E(2:len,:);
P_mim = Param_min.P(2:len,:);
AP_mim = Param_min.AP(2:len,:);
R12_min = Param_min.R12(2:len,:);
R13_min = Param_min.R13(2:len,:);
R23_min = Param_min.R23(2:len,:);

E_hour = Param_hour.E(2:len,:);
P_hour = Param_hour.P(2:len,:);
AP_hour = Param_hour.AP(2:len,:);
R12_hour = Param_hour.R12(2:len,:);
R13_hour = Param_hour.R13(2:len,:);
R23_hour = Param_hour.R23(2:len,:);

E_day = Param_day.E(2:len,:);
P_day = Param_day.P(2:len,:);
AP_day = Param_day.AP(2:len,:);
R12_day = Param_day.R12(2:len,:);
R13_day = Param_day.R13(2:len,:);
R23_day = Param_day.R23(2:len,:);

%% 绘图-能量
figure;

ax1 = subplot(2,3,1);
plot(E_mim(:,3),'color','r');hold on;plot(E_hour(:,3),'color','g');plot(E_day(:,3),'color','b');
xlabel('时间/天');ylabel('系统总能量/焦耳');title('系统总能量随时间的变化');
legend(ax1,'calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
deltaE_min = E_mim(:,3)-E_mim(1,3);
deltaErate_min = deltaE_min/abs(E_mim(1,3))*100;
deltaE_hour = E_hour(:,3)-E_hour(1,3);
deltaErate_hour = deltaE_hour/abs(E_hour(1,3))*100;
deltaE_day = E_day(:,3)-E_day(1,3);
deltaErate_day = deltaE_day/abs(E_day(1,3))*100;
plot(deltaErate_min,'color','r');hold on;plot(deltaErate_hour,'color','g');plot(deltaErate_day,'color','b');
xlabel('时间/天');ylabel('deltaE/E0 /%');title('系统总能量误差随时间的变化');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,2);
plot(E_mim(:,1),'color','r');hold on;plot(E_hour(:,1),'color','g');plot(E_day(:,1),'color','b');
xlabel('时间/天');ylabel('系统总动能/焦耳');title('系统总动能随时间的变化');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,5);
deltaE_min = E_mim(:,1)-E_mim(1,1);
deltaErate_min = deltaE_min/abs(E_mim(1,1))*100;
deltaE_hour = E_hour(:,1)-E_hour(1,1);
deltaErate_hour = deltaE_hour/abs(E_hour(1,1))*100;
deltaE_day = E_day(:,1)-E_day(1,1);
deltaErate_day = deltaE_day/abs(E_day(1,1))*100;
plot(deltaErate_min,'color','r');hold on;plot(deltaErate_hour,'color','g');plot(deltaErate_day,'color','b');
xlabel('时间/天');ylabel('deltaEk/Ek0 /%');title('系统总动能误差随时间的变化');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,3);
plot(E_mim(:,2),'color','r');hold on;plot(E_hour(:,2),'color','g');plot(E_day(:,2),'color','b');
xlabel('时间/天');ylabel('系统总引力势能/焦耳');title('系统总引力势能随时间的变化');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,6);
deltaE_min = E_mim(:,2)-E_mim(1,2);
deltaErate_min = deltaE_min/abs(E_mim(1,2))*100;
deltaE_hour = E_hour(:,2)-E_hour(1,2);
deltaErate_hour = deltaE_hour/abs(E_hour(1,2))*100;
deltaE_day = E_day(:,2)-E_day(1,2);
deltaErate_day = deltaE_day/abs(E_day(1,2))*100;
plot(deltaErate_min,'color','r');hold on;plot(deltaErate_hour,'color','g');plot(deltaErate_day,'color','b');
xlabel('时间/天');ylabel('deltaEg/Eg0 /%');title('系统总引力势能误差随时间的变化');
% legend('calculate per min','calculate per hour','calculate per day');

%% 绘图-动量
figure;

subplot(2,3,1);
% P_mim
plot3(P_mim(:,1),P_mim(:,2),P_mim(:,3),'color','r');grid on;hold on         % 轨迹
scatter3(P_mim(1,1),P_mim(1,2),P_mim(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r');    % 起点
scatter3(P_mim(end,1),P_mim(end,2),P_mim(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','r');  % 终点
% P_hour
plot3(P_hour(:,1),P_hour(:,2),P_hour(:,3),'color','g');grid on;hold on
scatter3(P_hour(1,1),P_hour(1,2),P_hour(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','g');
scatter3(P_hour(end,1),P_hour(end,2),P_hour(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','g');
% P_day
plot3(P_day(:,1),P_day(:,2),P_day(:,3),'color','b');grid on;hold on
scatter3(P_day(1,1),P_day(1,2),P_day(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','b');
scatter3(P_day(end,1),P_day(end,2),P_day(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','b');
% 补充
xlabel('X轴/ kg*m/s');ylabel('Y轴/ kg*m/s');zlabel('Z轴/ kg*m/s');title('动量随时间的变化轨迹');

subplot(2,3,2);
plot(P_mim(:,4),'color','r');hold on;plot(P_hour(:,4),'color','g');plot(P_day(:,4),'color','b');
xlabel('时间/天');ylabel('系统总动量/ kg*m/s');title('系统总动量随时间的变化');
legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,3);
deltaPrate_min = (P_mim(:,5)-P_mim(1,5))/P_mim(1,5)*100;
deltaPrate_hour = (P_hour(:,5)-P_hour(1,5))/P_hour(1,5)*100;
deltaPrate_day = (P_day(:,5)-P_day(1,5))/P_day(1,5)*100;
plot(deltaPrate_min,'color','r');hold on;plot(deltaPrate_hour,'color','g');plot(deltaPrate_day,'color','b');
xlabel('时间/天');ylabel('deltaABSP/ABSP0 /%');title('系统总绝对动量误差随时间的变化');

subplot(2,3,4);
plot(P_mim(:,1),'color','r');hold on;plot(P_hour(:,1),'color','g');plot(P_day(:,1),'color','b');
xlabel('时间/天');ylabel('系统X轴动量/ kg*m/s');title('系统X轴动量随时间的变化');

subplot(2,3,5);
plot(P_mim(:,2),'color','r');hold on;plot(P_hour(:,2),'color','g');plot(P_day(:,2),'color','b');
xlabel('时间/天');ylabel('系统Y轴动量/ kg*m/s');title('系统Y轴动量随时间的变化');

subplot(2,3,6);
plot(P_mim(:,3),'color','r');hold on;plot(P_hour(:,3),'color','g');plot(P_day(:,3),'color','b');
xlabel('时间/天');ylabel('系统Z轴动量/ kg*m/s');title('系统Z轴动量随时间的变化');

%% 绘图-角动量
figure;

subplot(2,2,1);
% AP_mim
plot3(AP_mim(:,1),AP_mim(:,2),AP_mim(:,3),'color','r');grid on;hold on;
scatter3(AP_mim(1,1),AP_mim(1,2),AP_mim(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r');
scatter3(AP_mim(end,1),AP_mim(end,2),AP_mim(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','r');
% AP_hour
plot3(AP_hour(:,1),AP_hour(:,2),AP_hour(:,3),'color','g');grid on;hold on;
scatter3(AP_hour(1,1),AP_hour(1,2),AP_hour(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','g');
scatter3(AP_hour(end,1),AP_hour(end,2),AP_hour(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','g');
% AP_day
plot3(AP_day(:,1),AP_day(:,2),AP_day(:,3),'color','b');grid on;hold on;
scatter3(AP_day(1,1),AP_day(1,2),AP_day(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','b');
scatter3(AP_day(end,1),AP_day(end,2),AP_day(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','b');
% 补充
xlabel('X轴/ kg*m^2/s');ylabel('Y轴/ kg*m^2/s');zlabel('Z轴/ kg*m^2/s');title('角动量随时间的变化轨迹');

subplot(2,2,2);
plot(AP_mim(:,4),'color','r');hold on;plot(AP_hour(:,4),'color','g');plot(AP_day(:,4),'color','b');
xlabel('时间/天');ylabel('系统总角动量/ kg*m^2/s');title('系统总角动量随时间的变化');
legend('calculate per min','calculate per hour','calculate per day');

subplot(2,4,5);
plot(AP_mim(:,1),'color','r');hold on;plot(AP_hour(:,1),'color','g');plot(AP_day(:,1),'color','b');
xlabel('时间/天');ylabel('系统X轴角动量/ kg*m^2/s');title('系统X轴角动量随时间的变化');

subplot(2,4,6);
plot(AP_mim(:,2),'color','r');hold on;plot(AP_hour(:,2),'color','g');plot(AP_day(:,2),'color','b');
xlabel('时间/天');ylabel('系统Y轴角动量/ kg*m^2/s');title('系统Y轴角动量随时间的变化');

subplot(2,4,7);
plot(AP_mim(:,3),'color','r');hold on;plot(AP_hour(:,3),'color','g');plot(AP_day(:,3),'color','b');
xlabel('时间/天');ylabel('系统Z轴角动量/ kg*m^2/s');title('系统Z轴角动量随时间的变化');

subplot(2,4,8);
deltaAP_min = AP_mim(:,3)-AP_mim(1,3);
deltaAPrate_min = deltaAP_min/abs(AP_mim(1,3))*100;
deltaAP_hour = AP_hour(:,3)-AP_hour(1,3);
deltaAPrate_hour = deltaAP_hour/abs(AP_hour(1,3))*100;
deltaAP_day = AP_day(:,3)-AP_day(1,3);
deltaAPrate_day = deltaAP_day/abs(AP_day(1,3))*100;
plot(deltaAPrate_min,'color','r');hold on;plot(deltaAPrate_hour,'color','g');plot(deltaAPrate_day,'color','b');
xlabel('时间/天');ylabel('deltaAP/AP0 /%');title('系统总角动量误差随时间的变化');

%% 绘图-质点间距
figure;

subplot(2,3,1);
plot(R12_min,'color','r');hold on;plot(R12_hour,'color','g');plot(R12_day,'color','b');
xlabel('时间/天');ylabel('间距/米');title('两星间距R12随时间的变化');
legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
R12rate_min = (R12_min-R12_min(1))/R12_min(1)*100;
R12rate_hour = (R12_hour-R12_hour(1))/R12_hour(1)*100;
R12rate_day = (R12_day-R12_day(1))/R12_day(1)*100;
plot(R12rate_min,'color','r');hold on;plot(R12rate_hour,'color','g');plot(R12rate_day,'color','b');
xlabel('时间/天');ylabel('间距变化量/%');title('两星间距R12的变化量随时间的变化');

subplot(2,3,2);
plot(R13_min,'color','r');hold on;plot(R13_hour,'color','g');plot(R13_day,'color','b');
xlabel('时间/天');ylabel('间距/米');title('两星间距R13随时间的变化');

subplot(2,3,5);
R13rate_min = (R13_min-R13_min(1))/R13_min(1)*100;
R13rate_hour = (R13_hour-R13_hour(1))/R13_hour(1)*100;
R13rate_day = (R13_day-R13_day(1))/R13_day(1)*100;
plot(R13rate_min,'color','r');hold on;plot(R13rate_hour,'color','g');plot(R13rate_day,'color','b');
xlabel('时间/天');ylabel('间距变化量/%');title('两星间距R13的变化量随时间的变化');

subplot(2,3,3);
plot(R23_min,'color','r');hold on;plot(R23_hour,'color','g');plot(R23_day,'color','b');
xlabel('时间/天');ylabel('间距/米');title('两星间距R23随时间的变化');

subplot(2,3,6);
R23rate_min = (R23_min-R23_min(1))/R23_min(1)*100;
R23rate_hour = (R23_hour-R23_hour(1))/R23_hour(1)*100;
R23rate_day = (R23_day-R23_day(1))/R23_day(1)*100;
plot(R23rate_min,'color','r');hold on;plot(R23rate_hour,'color','g');plot(R23rate_day,'color','b');
xlabel('时间/天');ylabel('间距变化量/%');title('两星间距R23的变化量随时间的变化');
