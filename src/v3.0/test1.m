%% ����׼��
% ���ݶ���
load([pwd,'\data\EPAP1_min1.mat'])
Param_min = Param;
load([pwd,'\data\EPAP1_hour1.mat'])
Param_hour = Param;
load([pwd,'\data\EPAP1_day1.mat'])
Param_day = Param;
% ��̼�¼ʱ��
% len = min([size(Param_min.E,1) size(Param_hour.E,1) size(Param_day.E,1)]);
len = 1001;
% ����������ı������
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

%% ��ͼ-����
figure;

ax1 = subplot(2,3,1);
plot(E_mim(:,3),'color','r');hold on;plot(E_hour(:,3),'color','g');plot(E_day(:,3),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳ������/����');title('ϵͳ��������ʱ��ı仯');
legend(ax1,'calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
deltaE_min = E_mim(:,3)-E_mim(1,3);
deltaErate_min = deltaE_min/abs(E_mim(1,3))*100;
deltaE_hour = E_hour(:,3)-E_hour(1,3);
deltaErate_hour = deltaE_hour/abs(E_hour(1,3))*100;
deltaE_day = E_day(:,3)-E_day(1,3);
deltaErate_day = deltaE_day/abs(E_day(1,3))*100;
plot(deltaErate_min,'color','r');hold on;plot(deltaErate_hour,'color','g');plot(deltaErate_day,'color','b');
xlabel('ʱ��/��');ylabel('deltaE/E0 /%');title('ϵͳ�����������ʱ��ı仯');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,2);
plot(E_mim(:,1),'color','r');hold on;plot(E_hour(:,1),'color','g');plot(E_day(:,1),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳ�ܶ���/����');title('ϵͳ�ܶ�����ʱ��ı仯');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,5);
deltaE_min = E_mim(:,1)-E_mim(1,1);
deltaErate_min = deltaE_min/abs(E_mim(1,1))*100;
deltaE_hour = E_hour(:,1)-E_hour(1,1);
deltaErate_hour = deltaE_hour/abs(E_hour(1,1))*100;
deltaE_day = E_day(:,1)-E_day(1,1);
deltaErate_day = deltaE_day/abs(E_day(1,1))*100;
plot(deltaErate_min,'color','r');hold on;plot(deltaErate_hour,'color','g');plot(deltaErate_day,'color','b');
xlabel('ʱ��/��');ylabel('deltaEk/Ek0 /%');title('ϵͳ�ܶ��������ʱ��ı仯');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,3);
plot(E_mim(:,2),'color','r');hold on;plot(E_hour(:,2),'color','g');plot(E_day(:,2),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳ����������/����');title('ϵͳ������������ʱ��ı仯');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,6);
deltaE_min = E_mim(:,2)-E_mim(1,2);
deltaErate_min = deltaE_min/abs(E_mim(1,2))*100;
deltaE_hour = E_hour(:,2)-E_hour(1,2);
deltaErate_hour = deltaE_hour/abs(E_hour(1,2))*100;
deltaE_day = E_day(:,2)-E_day(1,2);
deltaErate_day = deltaE_day/abs(E_day(1,2))*100;
plot(deltaErate_min,'color','r');hold on;plot(deltaErate_hour,'color','g');plot(deltaErate_day,'color','b');
xlabel('ʱ��/��');ylabel('deltaEg/Eg0 /%');title('ϵͳ���������������ʱ��ı仯');
% legend('calculate per min','calculate per hour','calculate per day');

%% ��ͼ-����
figure;

subplot(2,3,1);
% P_mim
plot3(P_mim(:,1),P_mim(:,2),P_mim(:,3),'color','r');grid on;hold on         % �켣
scatter3(P_mim(1,1),P_mim(1,2),P_mim(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r');    % ���
scatter3(P_mim(end,1),P_mim(end,2),P_mim(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','r');  % �յ�
% P_hour
plot3(P_hour(:,1),P_hour(:,2),P_hour(:,3),'color','g');grid on;hold on
scatter3(P_hour(1,1),P_hour(1,2),P_hour(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','g');
scatter3(P_hour(end,1),P_hour(end,2),P_hour(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','g');
% P_day
plot3(P_day(:,1),P_day(:,2),P_day(:,3),'color','b');grid on;hold on
scatter3(P_day(1,1),P_day(1,2),P_day(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','b');
scatter3(P_day(end,1),P_day(end,2),P_day(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','b');
% ����
xlabel('X��/ kg*m/s');ylabel('Y��/ kg*m/s');zlabel('Z��/ kg*m/s');title('������ʱ��ı仯�켣');

subplot(2,3,2);
plot(P_mim(:,4),'color','r');hold on;plot(P_hour(:,4),'color','g');plot(P_day(:,4),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳ�ܶ���/ kg*m/s');title('ϵͳ�ܶ�����ʱ��ı仯');
legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,3);
deltaPrate_min = (P_mim(:,5)-P_mim(1,5))/P_mim(1,5)*100;
deltaPrate_hour = (P_hour(:,5)-P_hour(1,5))/P_hour(1,5)*100;
deltaPrate_day = (P_day(:,5)-P_day(1,5))/P_day(1,5)*100;
plot(deltaPrate_min,'color','r');hold on;plot(deltaPrate_hour,'color','g');plot(deltaPrate_day,'color','b');
xlabel('ʱ��/��');ylabel('deltaABSP/ABSP0 /%');title('ϵͳ�ܾ��Զ��������ʱ��ı仯');

subplot(2,3,4);
plot(P_mim(:,1),'color','r');hold on;plot(P_hour(:,1),'color','g');plot(P_day(:,1),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳX�ᶯ��/ kg*m/s');title('ϵͳX�ᶯ����ʱ��ı仯');

subplot(2,3,5);
plot(P_mim(:,2),'color','r');hold on;plot(P_hour(:,2),'color','g');plot(P_day(:,2),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳY�ᶯ��/ kg*m/s');title('ϵͳY�ᶯ����ʱ��ı仯');

subplot(2,3,6);
plot(P_mim(:,3),'color','r');hold on;plot(P_hour(:,3),'color','g');plot(P_day(:,3),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳZ�ᶯ��/ kg*m/s');title('ϵͳZ�ᶯ����ʱ��ı仯');

%% ��ͼ-�Ƕ���
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
% ����
xlabel('X��/ kg*m^2/s');ylabel('Y��/ kg*m^2/s');zlabel('Z��/ kg*m^2/s');title('�Ƕ�����ʱ��ı仯�켣');

subplot(2,2,2);
plot(AP_mim(:,4),'color','r');hold on;plot(AP_hour(:,4),'color','g');plot(AP_day(:,4),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳ�ܽǶ���/ kg*m^2/s');title('ϵͳ�ܽǶ�����ʱ��ı仯');
legend('calculate per min','calculate per hour','calculate per day');

subplot(2,4,5);
plot(AP_mim(:,1),'color','r');hold on;plot(AP_hour(:,1),'color','g');plot(AP_day(:,1),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳX��Ƕ���/ kg*m^2/s');title('ϵͳX��Ƕ�����ʱ��ı仯');

subplot(2,4,6);
plot(AP_mim(:,2),'color','r');hold on;plot(AP_hour(:,2),'color','g');plot(AP_day(:,2),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳY��Ƕ���/ kg*m^2/s');title('ϵͳY��Ƕ�����ʱ��ı仯');

subplot(2,4,7);
plot(AP_mim(:,3),'color','r');hold on;plot(AP_hour(:,3),'color','g');plot(AP_day(:,3),'color','b');
xlabel('ʱ��/��');ylabel('ϵͳZ��Ƕ���/ kg*m^2/s');title('ϵͳZ��Ƕ�����ʱ��ı仯');

subplot(2,4,8);
deltaAP_min = AP_mim(:,3)-AP_mim(1,3);
deltaAPrate_min = deltaAP_min/abs(AP_mim(1,3))*100;
deltaAP_hour = AP_hour(:,3)-AP_hour(1,3);
deltaAPrate_hour = deltaAP_hour/abs(AP_hour(1,3))*100;
deltaAP_day = AP_day(:,3)-AP_day(1,3);
deltaAPrate_day = deltaAP_day/abs(AP_day(1,3))*100;
plot(deltaAPrate_min,'color','r');hold on;plot(deltaAPrate_hour,'color','g');plot(deltaAPrate_day,'color','b');
xlabel('ʱ��/��');ylabel('deltaAP/AP0 /%');title('ϵͳ�ܽǶ��������ʱ��ı仯');

%% ��ͼ-�ʵ���
figure;

subplot(2,3,1);
plot(R12_min,'color','r');hold on;plot(R12_hour,'color','g');plot(R12_day,'color','b');
xlabel('ʱ��/��');ylabel('���/��');title('���Ǽ��R12��ʱ��ı仯');
legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
R12rate_min = (R12_min-R12_min(1))/R12_min(1)*100;
R12rate_hour = (R12_hour-R12_hour(1))/R12_hour(1)*100;
R12rate_day = (R12_day-R12_day(1))/R12_day(1)*100;
plot(R12rate_min,'color','r');hold on;plot(R12rate_hour,'color','g');plot(R12rate_day,'color','b');
xlabel('ʱ��/��');ylabel('���仯��/%');title('���Ǽ��R12�ı仯����ʱ��ı仯');

subplot(2,3,2);
plot(R13_min,'color','r');hold on;plot(R13_hour,'color','g');plot(R13_day,'color','b');
xlabel('ʱ��/��');ylabel('���/��');title('���Ǽ��R13��ʱ��ı仯');

subplot(2,3,5);
R13rate_min = (R13_min-R13_min(1))/R13_min(1)*100;
R13rate_hour = (R13_hour-R13_hour(1))/R13_hour(1)*100;
R13rate_day = (R13_day-R13_day(1))/R13_day(1)*100;
plot(R13rate_min,'color','r');hold on;plot(R13rate_hour,'color','g');plot(R13rate_day,'color','b');
xlabel('ʱ��/��');ylabel('���仯��/%');title('���Ǽ��R13�ı仯����ʱ��ı仯');

subplot(2,3,3);
plot(R23_min,'color','r');hold on;plot(R23_hour,'color','g');plot(R23_day,'color','b');
xlabel('ʱ��/��');ylabel('���/��');title('���Ǽ��R23��ʱ��ı仯');

subplot(2,3,6);
R23rate_min = (R23_min-R23_min(1))/R23_min(1)*100;
R23rate_hour = (R23_hour-R23_hour(1))/R23_hour(1)*100;
R23rate_day = (R23_day-R23_day(1))/R23_day(1)*100;
plot(R23rate_min,'color','r');hold on;plot(R23rate_hour,'color','g');plot(R23rate_day,'color','b');
xlabel('ʱ��/��');ylabel('���仯��/%');title('���Ǽ��R23�ı仯����ʱ��ı仯');
