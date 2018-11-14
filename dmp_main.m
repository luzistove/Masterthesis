close all;
clear all;

%%  Import trajectory

% take simple sine function for example
% y = sin(t);
% dy = cos(t);
% ddy = -sin(t);
% y0 = y(1);
% yg = y(end);

load('modified5times.mat')
hipPitchL = mean([modified5times(1,:);modified5times(7,:);modified5times(13,:);modified5times(19,:);modified5times(25,:)]);
hipPitchR = mean([modified5times(2,:);modified5times(8,:);modified5times(14,:);modified5times(20,:);modified5times(26,:)]);
kneeBendL = -mean([modified5times(3,:);modified5times(9,:);modified5times(15,:);modified5times(21,:);modified5times(27,:)]);
kneeBendR = -mean([modified5times(4,:);modified5times(10,:);modified5times(16,:);modified5times(22,:);modified5times(28,:)]);
anklePitchL = mean([modified5times(5,:);modified5times(11,:);modified5times(17,:);modified5times(23,:);modified5times(29,:)]);
anklePitchR = mean([modified5times(6,:);modified5times(12,:);modified5times(18,:);modified5times(24,:);modified5times(30,:)]);
hipRollL = mean([modified5times(31,:);modified5times(35,:);modified5times(39,:);modified5times(43,:);modified5times(47,:)]);
hipRollR = mean([modified5times(32,:);modified5times(36,:);modified5times(40,:);modified5times(44,:);modified5times(48,:)]);
ankleRollL = mean([modified5times(33,:);modified5times(37,:);modified5times(41,:);modified5times(45,:);modified5times(49,:)]);
ankleRollR = mean([modified5times(34,:);modified5times(38,:);modified5times(42,:);modified5times(46,:);modified5times(50,:)]);

t = 0:0.01:12-0.01;
dt = 0.01;

angles = pchip(0:0.1:(12-0.1),[hipPitchL;hipPitchR;kneeBendL;kneeBendR;anklePitchL;anklePitchR;hipRollL;hipRollR;...
    ankleRollL;ankleRollR],t);

%% DMP
anglesIm = zeros(size(angles));

N_dmp = 10;      % number of dmps, number of curves to be imitated 
N_bf = 100;     % number of basic activation function
weights = zeros(N_dmp,N_bf);
ay = 25;    % ay & by: constants of the second order system
by = ay/4;
ax = 1;
tau = 10;

for i = 1:size(angles,1)
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

    fT = ddy-ay*(by*(yg-y)-dy);     % f_target

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
    
    for j = 1:length(t)
        if j == 1
            ddyIm(j) = (ay*(by*(yg-yIm(j))-dyIm(j))+f(j))/tau;
            dyIm(j) = dyIm(j)+ddyIm(j)*dt;
            yIm(j) = yIm(j)+dyIm(j)*dt;
        else
            ddyIm(j) = (ay*(by*(yg-yIm(j-1))-dyIm(j-1))+f(j))/tau;
            dyIm(j) = dyIm(j-1)+ddyIm(j)*dt;
            yIm(j) = yIm(j-1)+dyIm(j)*dt;
        end    
    end
    
    anglesIm(i,:) = yIm;
end

save('D:\Students\Zichong\dmp\weights','weights')
run('Nao_parameter')
sim('dmptest.slx')

%% Plotting
figure(1);
fs = 18;
fslegend = 20;
lw = 2;
subplot(5,2,1)
plot(t,angles(1,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(1,:),'b--','LineWidth',lw)
grid on
% xlabel('time[s]')
ylabel('angle[rad]')

title('Left hip pitch angle')
set(gca,'fontsize',fs)
% set(l1,'fontsize',fslegend)
subplot(5,2,2)
plot(t,angles(2,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(2,:),'b--','LineWidth',lw)
grid on
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Right hip pitch angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,3)
plot(t,angles(3,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(3,:),'b--','LineWidth',lw)
grid on
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Left knee bending angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,4)
plot(t,angles(4,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(4,:),'b--','LineWidth',lw)
grid on
% xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','SouthEast');
title('Right knee bending angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)
subplot(5,2,5)
plot(t,angles(5,:),'r','LineWidth',lw)
hold on
plot(t,anglesIm(5,:),'b--','LineWidth',lw)
grid on
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
grid on
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
grid on
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
grid on
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
grid on
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
grid on
xlabel('time[s]')
ylabel('angle[rad]')
% l2 = legend('Desired trajectory','Trajectory by imitation','Location','NorthEast');
title('Right ankle roll angle')
set(gca,'fontsize',fs)
% set(l2,'fontsize',fslegend)

l1 = legend('Desired trajectory','Trajectory by imitation','Location',[0.5 0.95 0.05 0.05],'Orientation','horizontal');
set(l1,'fontsize',fslegend)