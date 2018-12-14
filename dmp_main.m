% close all;
% clear all;

%%  Import trajectory

% take simple sine function for example
% y = sin(t);
% dy = cos(t);
% ddy = -sin(t);
% y0 = y(1);
% yg = y(end);
% % 
% load('straightWalk5.mat')
% temp1 = straightWalk5;
% load('kick5.mat')
% temp1 = kick5;
load('sideWalk5.mat')
temp1 = sideWalk5;
% load('primiKick')
%%


hipPitchL = mean([temp1(1,:);temp1(11,:);temp1(21,:);temp1(31,:);temp1(41,:)]);
hipPitchR = mean([temp1(2,:);temp1(12,:);temp1(22,:);temp1(32,:);temp1(42,:)]);
kneeBendL = -mean([temp1(3,:);temp1(13,:);temp1(23,:);temp1(33,:);temp1(43,:)]);
kneeBendR = -mean([temp1(4,:);temp1(14,:);temp1(24,:);temp1(34,:);temp1(44,:)]);
anklePitchL = mean([temp1(5,:);temp1(15,:);temp1(25,:);temp1(35,:);temp1(45,:)]);
anklePitchR = mean([temp1(6,:);temp1(16,:);temp1(26,:);temp1(36,:);temp1(46,:)]);
hipRollL = mean([temp1(7,:);temp1(17,:);temp1(27,:);temp1(37,:);temp1(47,:)]);
hipRollR = mean([temp1(8,:);temp1(18,:);temp1(28,:);temp1(38,:);temp1(48,:)]);
ankleRollL = mean([temp1(9,:);temp1(19,:);temp1(29,:);temp1(39,:);temp1(49,:)]);
ankleRollR = mean([temp1(10,:);temp1(20,:);temp1(30,:);temp1(40,:);temp1(50,:)]);
% 
dt = 0.01;
t = 0:dt:0.1*size(temp1,2)-0.1;


angles = pchip(0:0.1:0.1*size(temp1,2)-0.1,[hipPitchL;hipPitchR;kneeBendL;kneeBendR;anklePitchL;...
    anklePitchR;hipRollL;hipRollR;ankleRollL;ankleRollR],t);
%% Trajectory cutting, motion primitives
% straight walking
% angles(:,551:end) = [];
% angles(:,1:344) = [];

% kick
% angles(:,690:end) = [];
% angles(:,1:410) = [];

% side walking right
% angles(:,906:end) = [];
% angles(:,1:741) = [];

% side walking left
% angles(:,763:end) = [];
% angles(:,1:687) = [];

% first half step
% angles(:,356:end) = [];
% angles(:,1:219) = [];

% final half step
% angles(:,1051:end) = [];
% angles(:,1:849) = [];

% right step 
% angles(:,556:end) = [];
% angles(:,1:444) = [];

% left step
% angles(:,446:end) = [];
% angles(:,1:339) = [];

% wohle side
angles(:,630:end) = [];
angles(:,1:415) = [];

dt = 0.01;
t = 0:dt:0.01*(size(angles,2)-1);
%% DMP
anglesIm = zeros(size(angles));

N_dmp = 10;      % number of dmps, number of curves to be imitated 
N_bf = 50;     % number of basic activation function
weights = zeros(N_dmp,N_bf);
ay = 25;    % ay & by: constants of the second order system
by = ay/4;


for i = 1:10
    y = angles(i,:);
    y0 = y(1); yg = y(end);
    
    
    % check if the initial position and the goal are the same, if so, give
    % slight offset so that the forcing term is never 0
    if y0 == yg
        yg = yg+0.001;
    end
    
    % 1st and 2nd derivative
    y1 = [y y(end) y(end)];
    dy = zeros(1,length(y1)-1);
    ddy = zeros(1,length(dy)-1);

    for j = 1:length(dy)
        dy(1,j) = (y1(1,j+1)-y1(1,j))/dt;
    end

    for j = 1:length(ddy)
        ddy(1,j) = (dy(1,j+1)-dy(1,j))/dt;
    end
    dy(end) = [];
    
    % Canonical system
    ax = 1;
    tau = 1;
    x = exp(-ax/tau*t);
    
    % Generating basic functions
    des_c = linspace(0,max(t),N_bf);    % point in time where to put the activation function
    c = exp(-ax/tau*des_c);             % point in x where to put the activation function
    h = N_bf^1.5*ones(1,N_bf)./c;       % variance of each activation function, trial and error


    psi = zeros(N_bf,length(t));
    for j = 1:N_bf
        for k = 1:length(x)
            psi(j,k) = exp(-h(j)*(x(k)-c(j))^2);
        end
    end    
    
    % Weights

    fT = tau*ddy-ay*(by*(yg-y)-dy);     % f_target

    w = zeros(1,N_bf);  % weights in the forcing term
    ksi = (yg-y0)*x;
    for j = 1:N_bf
        w(j) = (ksi*diag(psi(j,:))*fT')/(ksi*diag(psi(j,:))*ksi');
    end
    weights(i,:) = w;
    
    % Forcing term
    wpsi = w*psi;
    f = wpsi./(sum(psi)).*x*(yg-y0);
    
    % Reproducing
    ddyIm = zeros(1,length(t));
    dyIm = zeros(1,length(t));
    yIm = zeros(1,length(t));
    yIm(1) = y0;
    
    for k = 2:length(t)
            ddyIm(k-1) = 1/tau^2*(ay*(by*(yg-yIm(k-1))-tau*dyIm(k-1))+f(k-1));
            dyIm(k) = dyIm(k-1)+ddyIm(k-1)*dt;
            yIm(k) = yIm(k-1)+dyIm(k-1)*dt;
   
    end
    
    
    anglesIm(i,:) = yIm;
end

%% Time scaling
% tau1 = 1.5;
% t1 = 0:0.01:tau1*t(:,end);
% anglesIm1 = zeros(10,length(t1));
% 
% for i = 1:10
%     ax = 1;
%     t1 = 0:0.01:tau1*t(:,end);
%     x1 = exp(-ax/tau1*t1);
%     
%     % Generating basic functions
%     des_c1 = linspace(0,max(t1),N_bf);    % point in time where to put the activation function
%     c1 = exp(-ax/tau1*des_c);             % point in x where to put the activation function
%     h1 = N_bf^1.5*ones(1,N_bf)./c1;       % variance of each activation function, trial and error
% 
% 
%     psi1 = zeros(N_bf,length(t1));
%     for j = 1:N_bf
%         for k = 1:length(x1)
%             psi1(j,k) = exp(-h1(j)*(x1(k)-c1(j))^2);
%         end
%     end    
%    
%     
%     % Forcing term
%     wpsi1 = weights(i,:)*psi1;
%     f1(i,:) = wpsi1./(sum(psi1)).*x1*(yg-y0);
%     
%     % Reproducing
%     ddyIm = zeros(1,length(0:dt:tau1*t(end)));
%     dyIm = zeros(1,length(0:dt:tau1*t(end)));
%     yIm = zeros(1,length(0:dt:tau1*t(end)));
%     yIm(1) = y0;
%     
%     for j = 1:length(t1)
%         if j == 1
%             ddyIm(j) = 1/tau1^2*(ay*(by*(yg-yIm(j))-tau1*dyIm(j))+f1(i,j));
%             dyIm(j) = dyIm(j)+ddyIm(j)*dt;
%             yIm(j) = yIm(j)+dyIm(j)*dt;
%         else
%             ddyIm(j) = 1/tau1^2*(ay*(by*(yg-yIm(j-1))-tau1*dyIm(j-1))+f1(i,j));
%             dyIm(j) = dyIm(j-1)+ddyIm(j)*dt;
%             yIm(j) = yIm(j-1)+dyIm(j)*dt;
%         end    
%     end
%     
%     anglesIm1(i,:) = yIm;
% end
% figure(2)
% plot(t1,anglesIm1(10,:))
% hold on
% plot(t,anglesIm(1,:))


%% Plotting
figure(1);
fs = 15;
fslegend = 20;
lw = 2;
subplot(5,2,1)
plot(t,angles(1,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(1,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(1,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(1,:),'g--','LineWidth',lw)
grid on
% xlabel('time[s]')
ylabel('angle[rad]')
title('Left hip pitch angle')
axis tight

set(gca,'fontsize',fs)
% set(l1,'fontsize',fslegend)
subplot(5,2,2)
plot(t,angles(2,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(2,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(2,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(2,:),'g--','LineWidth',lw)
grid on
% xlabel('time[s]')
ylabel('angle[rad]')
axis tight
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Right hip pitch angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,3)
plot(t,angles(3,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(3,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(3,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(3,:),'g--','LineWidth',lw)
grid on
axis tight
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Left knee bend angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,4)
plot(t,angles(4,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(4,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(4,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(4,:),'g--','LineWidth',lw)
grid on
axis tight
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Right knee bend angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,5)
plot(t,angles(5,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(5,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(5,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(5,:),'g--','LineWidth',lw)
grid on
axis tight
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','NorthEast');
title('Left ankle pitch angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,6)
plot(t,angles(6,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(6,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(6,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(6,:),'g--','LineWidth',lw)
grid on
axis tight
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','NorthEast');
title('Right ankle pitch angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,7)
plot(t,angles(7,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(7,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(7,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(7,:),'g--','LineWidth',lw)
grid on
axis tight
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Left hip roll angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,8)
plot(t,angles(8,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(8,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(8,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(8,:),'g--','LineWidth',lw)
grid on
axis tight
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Right hip roll angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,9)
plot(t,angles(9,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(9,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(9,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(9,:),'g--','LineWidth',lw)
grid on
axis tight
xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','NorthEast');
title('Left ankle roll angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,10)
plot(t,angles(10,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(10,:),'b--','LineWidth',lw)
% hold on
% plot(t,anglesIm1000(10,:),'m--','LineWidth',lw)
% hold on
% plot(t,anglesIm10000(10,:),'g--','LineWidth',lw)
grid on
axis tight%([0,1.05,-0.15,0.1])
xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','NorthEast');
title('Right ankle roll angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
% 
l1 = legend('Desired trajectory','Learned trajectory with 50 bfs','Location',[0.5 0.95 0.05 0.05],'Orientation','horizontal');
set(l1,'fontsize',fslegend)