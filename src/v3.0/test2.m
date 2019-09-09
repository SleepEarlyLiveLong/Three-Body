%% ����׼��
% ���ݶ���
load([pwd,'\data\EPAP5.mat'])

% ����������ı������
E = Param.E(2:min(1001,end),:);
P = Param.P(2:min(1001,end),:);
AP = Param.AP(2:min(1001,end),:);
R12 = Param.R12(2:min(1001,end),:);
R13 = Param.R13(2:min(1001,end),:);
R23 = Param.R23(2:min(1001,end),:);

%% ��ͼ-����
figure;

ax1 = subplot(2,3,1);
plot(E(:,3),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳ������/����');title('ϵͳ��������ʱ��ı仯');
% legend(ax1,'calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
deltaE = E(:,3)-E(1,3);
deltaErate = deltaE/abs(E(1,3))*100;
plot(deltaErate,'color','r');
xlabel('ʱ��/��');ylabel('deltaE/E0 /%');title({'ϵͳ�������䶯����';'��ʱ��ı仯'});

subplot(2,3,2);
plot(E(:,1),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳ�ܶ���/����');title('ϵͳ�ܶ�����ʱ��ı仯');

subplot(2,3,5);
deltaE = E(:,1)-E(1,1);
deltaErate = deltaE/abs(E(1,1))*100;
plot(deltaErate,'color','r');
xlabel('ʱ��/��');ylabel('deltaEk/Ek0 /%');title({'ϵͳ�ܶ��ܱ䶯����';'��ʱ��ı仯'});

subplot(2,3,3);
plot(E(:,2),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳ����������/����');title({'ϵͳ����������';'��ʱ��ı仯'});

subplot(2,3,6);
deltaE = E(:,2)-E(1,2);
deltaErate = deltaE/abs(E(1,2))*100;
plot(deltaErate,'color','r');
xlabel('ʱ��/��');ylabel('deltaEg/Eg0 /%');title({'ϵͳ���������ܱ䶯����';'��ʱ��ı仯'});

%% ��ͼ-����
figure;

subplot(2,3,1);
plot3(P(:,1),P(:,2),P(:,3),'color','r');grid on;hold on         % �켣
scatter3(P(1,1),P(1,2),P(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r');    % ���
scatter3(P(end,1),P(end,2),P(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','r');  % �յ�
% ����
xlabel('X��/ kg*m/s');ylabel('Y��/ kg*m/s');zlabel('Z��/ kg*m/s');title('������ʱ��ı仯�켣');

subplot(2,3,2);
plot(P(:,4),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳ�ܶ���/ kg*m/s');title({'ϵͳ�ܶ���';'��ʱ��ı仯'});
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,3);
deltaPrate = (P(:,5)-P(1,5))/P(1,5)*100;
plot(deltaPrate,'color','r');
xlabel('ʱ��/��');ylabel('deltaABSP/ABSP0 /%');title({'ϵͳ�ܾ��Զ������';'��ʱ��ı仯'});

subplot(2,3,4);
plot(P(:,1),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳX�ᶯ��/ kg*m/s');title({'ϵͳX�ᶯ��';'��ʱ��ı仯'});

subplot(2,3,5);
plot(P(:,2),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳY�ᶯ��/ kg*m/s');title({'ϵͳY�ᶯ��';'��ʱ��ı仯'});

subplot(2,3,6);
plot(P(:,3),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳZ�ᶯ��/ kg*m/s');title({'ϵͳZ�ᶯ��';'��ʱ��ı仯'});

%% ��ͼ-�Ƕ���
figure;

subplot(2,2,1);
plot3(AP(:,1),AP(:,2),AP(:,3),'color','r');grid on;hold on;
scatter3(AP(1,1),AP(1,2),AP(1,3),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r');
scatter3(AP(end,1),AP(end,2),AP(end,3),'Marker','s','MarkerEdgeColor','k','MarkerFaceColor','r');
legend('trace','start point','stop point');
% ����
xlabel('X��/ kg*m^2/s');ylabel('Y��/ kg*m^2/s');zlabel('Z��/ kg*m^2/s');title('�Ƕ�����ʱ��ı仯�켣');

subplot(2,2,2);
plot(AP(:,4),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳ�ܽǶ���/ kg*m^2/s');title({'ϵͳ�ܽǶ���';'��ʱ��ı仯'});

subplot(2,4,5);
plot(AP(:,1),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳX��Ƕ���/ kg*m^2/s');title({'ϵͳX��Ƕ���';'��ʱ��ı仯'});

subplot(2,4,6);
plot(AP(:,2),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳY��Ƕ���/ kg*m^2/s');title({'ϵͳY��Ƕ���';'��ʱ��ı仯'});

subplot(2,4,7);
plot(AP(:,3),'color','r');
xlabel('ʱ��/��');ylabel('ϵͳZ��Ƕ���/ kg*m^2/s');title({'ϵͳZ��Ƕ���';'��ʱ��ı仯'});

subplot(2,4,8);
deltaAP = AP(:,4)-AP(1,4);
deltaAPrate = deltaAP/abs(AP(1,4))*100;
plot(deltaAPrate,'color','r');
xlabel('ʱ��/��');ylabel('deltaAP/AP0 /%');title({'ϵͳ�ܽǶ������';'��ʱ��ı仯'});

%% ��ͼ-�ʵ���
figure;

subplot(2,3,1);
plot(R12,'color','r');
xlabel('ʱ��/��');ylabel('���/��');title('���Ǽ��R12��ʱ��ı仯');
% legend('calculate per min','calculate per hour','calculate per day');

subplot(2,3,4);
R12rate = (R12-R12(1))/R12(1)*100;
plot(R12rate,'color','r');
xlabel('ʱ��/��');ylabel('���仯��/%');title('���Ǽ��R12�ı仯����ʱ��ı仯');

subplot(2,3,2);
plot(R13,'color','r');
xlabel('ʱ��/��');ylabel('���/��');title('���Ǽ��R13��ʱ��ı仯');

subplot(2,3,5);
R13rate = (R13-R13(1))/R13(1)*100;
plot(R13rate,'color','r');
xlabel('ʱ��/��');ylabel('���仯��/%');title('���Ǽ��R13�ı仯����ʱ��ı仯');

subplot(2,3,3);
plot(R23,'color','r');
xlabel('ʱ��/��');ylabel('���/��');title('���Ǽ��R23��ʱ��ı仯');

subplot(2,3,6);
R23rate = (R23-R23(1))/R23(1)*100;
plot(R23rate,'color','r');
xlabel('ʱ��/��');ylabel('���仯��/%');title('���Ǽ��R23�ı仯����ʱ��ı仯');
