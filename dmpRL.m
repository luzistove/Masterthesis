

clear all;
%% RL process
% Reinforcement learning is a learning process of many rollouts. 1 rollout is 1 time weights update, 1 time DMP reproduction 
% with DMPs, 1 time simulation and 1 evaluation
N_dmp = 10;
N_dmp1 = 2;
N_bf = 50;
% load('E:\TUHH master\Master thesis\Code\dmp\primiRL\straighttrain\wLeftstepRL.mat');
% load('E:\TUHH master\Master thesis\Code\dmp\primiRL\straighttrain\wRightstepRL.mat');
% load('E:\TUHH master\Master thesis\Code\dmp\primiRL\straighttrain\primiLeftstepRL.mat');
% load('E:\TUHH master\Master thesis\Code\dmp\primiRL\straighttrain\primiRightstepRL.mat');
load('D:\Students\Zichong\dmp\primiRL\straighttrain\wLeftstepRL.mat');
load('D:\Students\Zichong\dmp\primiRL\straighttrain\wRightstepRL.mat');
load('D:\Students\Zichong\dmp\primiRL\straighttrain\primiLeftstepRL.mat');
load('D:\Students\Zichong\dmp\primiRL\straighttrain\primiRightstepRL.mat');
ax = 1;
tau = 1;
ay = 25; by = ay/4;
dt = 0.01;
t = 0:0.01:0.01*(size(primiLeftstep,2)-1);
%%
maxIt = 2;   % max number of rollouts. 
sig = 50;
mu = 0;
count = 1;
cap = 10;   % capacity of the importance sampler
R = ones(1,cap);     % Rewards
rolloutL = zeros(N_bf,cap*N_dmp1);
rolloutR = zeros(N_bf,cap*N_dmp1);
explore = zeros(N_bf,cap*N_dmp1);
tol = 1e-5;
wLeftstepold = zeros(N_dmp1,N_bf);
wRightstepold = zeros(N_dmp1,N_bf);
p1 = 7; p2 = p1+N_dmp1-1;
%% RL process
% wStraightRL = wStraightRL;
run('Nao_parameter')
while (norm(wLeftstep(p1:p2,:)-wLeftstepold) >= tol )%(norm(wRightstep(p1:p2,:)-wRightstepold) >= tol )%count <= 100%(norm(wRightstep(p1:p2,:)-wRightstepold) >= tol )% &%3
    temp = 2*normrnd(mu,sig,[N_dmp1,N_bf]);
    
    wLeftstep(p1:p2,:) = wLeftstep(p1:p2,:)+ temp;
    wRightstep(p1:p2,:) = wRightstep(p1:p2,:)+ temp;
    save('D:\Students\Zichong\dmp\primiRL\straighttrain\wLeftstepRL.mat','wLeftstep');
%     save('D:\Students\Zichong\dmp\primiRL\straighttrain\wRightstepRL.mat','wRightstep');
%     anglesIm = zeros(N_dmp,size(primiSideRL,2));
%     for i = 1:N_dmp
%         y = primiSideRL(i,:);
%         y0 = y(1); yg = y(end);
%     
%     
%         % check if the initial position and the goal are the same, if so, give
%         % slight offset so that the forcing term is never 0
%         if y0 == yg
%             yg = yg+0.001;
%         end
% 
%     
%         % Canonical system
%         ax = 1;
%         tau = 1;
%         x = exp(-ax/tau*t);
%     
%         % Generating basic functions
%         des_c = linspace(0,max(t),N_bf);    % point in time where to put the activation function
%         c = exp(-ax/tau*des_c);             % point in x where to put the activation function
%         h = N_bf^1.5*ones(1,N_bf)./c;       % variance of each activation function, trial and error
%       
%         psi = zeros(N_bf,length(t));
%         for j = 1:N_bf
%             for k = 1:length(x)
%                 psi(j,k) = exp(-h(j)*(x(k)-c(j))^2);
%             end
%         end   
% 
%         
%         wpsi = wSideRLtrain2*psi;
%         f = wpsi(i,:)./(sum(psi)).*x*(yg-y0);
%     
%         % Reproducing
%         ddyIm = zeros(1,length(t));
%         dyIm = zeros(1,length(t));
%         yIm = zeros(1,length(t));
%         yIm(1) = y0;
% 
%         for k = 2:length(t)
%             ddyIm(k-1) = 1/tau^2*(ay*(by*(yg-yIm(k-1))-tau*dyIm(k-1))+f(k-1));
%             dyIm(k) = dyIm(k-1)+ddyIm(k-1)*dt;
%             yIm(k) = yIm(k-1)+dyIm(k-1)*dt;
% 
%         end
%         anglesIm(i,:) = yIm;
%         
%     end
%     hipPitchL = anglesIm(1,:);
%     hipPitchR = anglesIm(2,:);
%     kneeBendL = anglesIm(3,:);
%     kneeBendR = anglesIm(4,:);
%     anklePitchL = -anglesIm(1,:)+anglesIm(3,:);
%     anklePitchR = -anglesIm(2,:)+anglesIm(4,:);
%     hipRollL = anglesIm(7,:);
%     hipRollR = anglesIm(8,:);
%     ankleRollL = -anglesIm(7,:);
%     ankleRollR = -anglesIm(8,:);
%     
    sim('dmpRLtest.slx')    % run simulation and get results
    posEnd(:,count) = position(end,2:4)';
    %% Evaluation
    
    % Stability
    for i = 1:size(position,1)
        if position(i,4) < 0.25
            fall = 1;
            break;
        else
            fall = 0;
        end
    end
    if fall == 1
        reFall = -10000;
    else
        reFall = 0;
    end
    
    % Whether still forward or backward offset
    if abs(position(end,3)) < 0.01
        reOff = 0;%20000*abs((position(end,2)-position(1,2)));
    else
        reOff = -2000*abs((position(end,3)-position(1,3)));
    end
    angles = angles';
    angles(1,:) = [];
    % Whether angle is beyond limit
    for i = 1:10
        switch i
            case 1
                if (min(angles(i,:)) <= deg2rad(-60) || max(angles(i,:)) >= deg2rad(60))
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 2
                if (min(angles(i,:)) <= rad2deg(-60) || max(angles(i,:)) >= rad2deg(60))
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 3
                if (min(angles(i,:)) <= -0.6 || max(angles(i,:)) > 0)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 4
                if (min(angles(i,:)) <= -0.6 || max(angles(i,:)) > 0)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 5
                angBeyond = 0;
            case 6
                angBeyond = 0;
            case 7
                if (min(angles(i,:)) < -0.3 || max(angles(i,:)) > 0.3)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 8
                if (min(angles(i,:)) < -0.3 || max(angles(i,:)) > 0.3)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end                
             case 9
                 angBeyond = 0;
             case 10
                 angBeyond = 0;    
        end
    end
    if angBeyond == 1
        reAng = -10000;
    else
        reAng = 0;
    end
    % Speed
%     reSpeed = abs((position(1,3)-position(end,3)))*(100);
    if abs((position(1,2)-position(end,2))) < 2.5
        reSpeed = -1000;%100*exp(abs((position(1,2)-position(end,2))));
    else 
        reSpeed = 0;
    end
    % Direction
    if position(end,3) > position(1,3)
        reDirection = 0;
    else 
        reDirection = 10;
    end
    
    
    % Oscillation if CoMy
    for j = 1:size(CoMy,2)
        if abs(CoMy(2,j)) >= 0.005
            oscy = 1;
            break;
        else
            oscy = 0;
        end
    end
    if oscy == 1
        reCoMy = -1000;
    else
        reCoMy = 0;
    end
    
    
    % simulation ends by error
    if stop(end) == 1
        reEnd = -10000;
    else
        reEnd = 0;
    end
    
    reAll = reFall+reOff+reEnd+reAng+reSpeed+reCoMy;%+reDirection;
    
    if count <= cap
        R(1,cap-count+1) = reAll;
%         rollout(:,(count-1)*N_dmp1+1:(count-1)*N_dmp1+N_dmp1) = wStraightRL(p1:p2,:)';
%         explore(:,(count-1)*N_dmp1+1:(count-1)*N_dmp1+N_dmp1) = temp';
        rolloutL(:,(cap-count)*N_dmp1+1:(cap-count)*N_dmp1+N_dmp1) = wLeftstep(p1:p2,:)';
        rolloutR(:,(cap-count)*N_dmp1+1:(cap-count)*N_dmp1+N_dmp1) = wRightstep(p1:p2,:)';
        explore(:,(cap-count)*N_dmp1+1:(cap-count)*N_dmp1+N_dmp1) = temp';
    else
        for i = 1:cap
            if reAll > R(1,i)
                for j = cap:-1:i+1
                    R(1,j) = R(1,j-1);
                    rolloutL(:,(j-1)*N_dmp1+1:(j-1)*N_dmp1+N_dmp1) = rolloutL(:,(j-2)*N_dmp1+1:(j-2)*N_dmp1+N_dmp1);
                    rolloutR(:,(j-1)*N_dmp1+1:(j-1)*N_dmp1+N_dmp1) = rolloutR(:,(j-2)*N_dmp1+1:(j-2)*N_dmp1+N_dmp1);
                    explore(:,(j-1)*N_dmp1+1:(j-1)*N_dmp1+N_dmp1) = explore(:,(j-2)*N_dmp1+1:(j-2)*N_dmp1+N_dmp1);
                end      
    %             R(1,i) = reAll;
    %             rollout(:,i*10+1:i*10+10) = wStraightRL';
                break;

            end
        end
        R(1,i) = reAll;
        rolloutL(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1) = wLeftstep(p1:p2,:)';
        rolloutR(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1) = wRightstep(p1:p2,:)';
    end
    
    R
    
    
    numL = zeros(N_bf,N_dmp1);
    denL = zeros(N_bf,N_dmp1);
    numR = zeros(N_bf,N_dmp1);
    denR = zeros(N_bf,N_dmp1);
    for i=1:cap
        numL = numL+((rolloutL(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1)-wLeftstep(p1:p2,:)').*R(1,i));
        denL = denL+R(:,i);
        numR = numR+((rolloutR(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1)-wRightstep(p1:p2,:)').*R(1,i));
        denR = denR+R(:,i);
%         num = num+(explore(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1).*R(1,i));
%         den = den+R(:,i);
    end
    % update 
    updateL = (numL./(denL+1.e-5))';
    updateR = (numR./(denL+1.e-5))';
    wLeftstepold = wLeftstep(p1:p2,:);
    wRightstepold = wRightstep(p1:p2,:);
    wLeftstep(p1:p2,:) = wLeftstep(p1:p2,:)+updateL;
    wRightstep(p1:p2,:) = wRightstep(p1:p2,:)+updateR;
%     figure(2)
%     plot(wStraightRL(:,8))
    count = count+1

    
end
%% Comparing
% wSideRLtrained = wSideRLtrain1;
%     anglesImt = zeros(N_dmp,size(primiStrRL,2));
%     for i = 1:N_dmp
%         y = primiStrRL(i,:);
%         y0 = y(1); yg = y(end);
%     
%     
%         check if the initial position and the goal are the same, if so, give
%         slight offset so that the forcing term is never 0
%         if y0 == yg
%             yg = yg+0.001;
%         end
% 
%     
%         Canonical system
%         ax = 1;
%         tau = 1;
%         x = exp(-ax/tau*t);
%     
%         Generating basic functions
%         des_c = linspace(0,max(t),N_bf);    % point in time where to put the activation function
%         c = exp(-ax/tau*des_c);             % point in x where to put the activation function
%         h = N_bf^1.5*ones(1,N_bf)./c;       % variance of each activation function, trial and error
%       
%         psi = zeros(N_bf,length(t));
%         for j = 1:N_bf
%             for k = 1:length(x)
%                 psi(j,k) = exp(-h(j)*(x(k)-c(j))^2);
%             end
%         end   
% 
%         
%         wpsi = wStraightRLtrain3*psi;
%         f = wpsi(i,:)./(sum(psi)).*x*(yg-y0);
%     
%         Reproducing
%         ddyIm = zeros(1,length(t));
%         dyIm = zeros(1,length(t));
%         yIm = zeros(1,length(t));
%         yIm(1) = y0;
% 
%         for k = 2:length(t)
%             ddyIm(k-1) = 1/tau^2*(ay*(by*(yg-yIm(k-1))-tau*dyIm(k-1))+f(k-1));
%             dyIm(k) = dyIm(k-1)+ddyIm(k-1)*dt;
%             yIm(k) = yIm(k-1)+dyIm(k-1)*dt;
% 
%         end
%         anglesImt(i,:) = yIm;
%         
%     end

%% Plotting
% numbers of iterative and offset in x direction
% c1 = 171;
% plot(1:c1,posEnd(1,:),'r','LineWidth',2)
% grid on
% xlabel('number of iteration')
% ylabel('position [m]')
% axis([1,171,-0.05,0.2])
% set(gca,'fontsize',20)

% capacity to iteration number
% cap 10 iteration 171
% cap 100 iteration 2170
% cap 50 iteration 861

% plot([10 50 100],[171 861 2170],'r-s','LineWidth',2)

% plot(1:300,posEnd(2,:),'r','LineWidth',2)
% grid on
% xlabel('numbers of iteration')
% ylabel('position [m]')
% set(gca,'fontsize',20)
% plot([10 50 100],[171 861 2170],'r-s','LineWidth',2)

% plot(1:219,posEndoff(1,:),'r','LineWidth',2)
% grid on
% xlabel('numbers of iteration')
% ylabel('position [m]')
% axis([0 219 -0.05 0.2])
% set(gca,'fontsize',20)
% plot(1:300,posEnd(1,:),'r','LineWidth',2)
% grid on
% xlabel('numbers of iteration')
% ylabel('position [m]')
% set(gca,'fontsize',20)

%%
% anglesImt(5,:) = -anglesImt(1,:)+anglesImt(3,:);
% anglesImt(6,:) = -anglesImt(2,:)+anglesImt(4,:);
% anglesImt(9,:) = -anglesImt(7,:);
% anglesImt(10,:) = -anglesImt(8,:);
% fs = 15;
% fslegend = 20;
% figure(1)
% subplot(5,2,1)
% plot(t,anglesIm(1,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(1,:),'g','LineWidth',2)
% grid on
% axis tight
% ylabel('angle [rad]')
% title('Left hip pitch angle')
% set(gca,'fontsize',fs)
% subplot(5,2,2)
% plot(t,anglesIm(2,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(2,:),'g','LineWidth',2)
% grid on
% axis tight
% ylabel('angle [rad]')
% title('Right hip pitch angle')
% set(gca,'fontsize',fs)
% subplot(5,2,3)
% plot(t,anglesIm(3,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(3,:),'g','LineWidth',2)
% grid on
% axis([0 2.05 -0.6 0.1])
% ylabel('angle [rad]')
% title('Left knee bend angle')
% set(gca,'fontsize',fs)
% subplot(5,2,4)
% plot(t,anglesIm(4,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(4,:),'g','LineWidth',2)
% grid on
% axis([0 2.05 -0.7 -0.08])
% ylabel('angle [rad]')
% title('Right knee bend angle')
% set(gca,'fontsize',fs)
% subplot(5,2,5)
% plot(t,anglesIm(5,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(5,:),'g','LineWidth',2)
% grid on
% axis tight
% ylabel('angle [rad]')
% title('Left ankle pitch angle')
% set(gca,'fontsize',fs)
% subplot(5,2,6)
% plot(t,anglesIm(6,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(6,:),'g','LineWidth',2)
% grid on
% axis([0 2.05 -0.7 0.5])
% ylabel('angle [rad]')
% title('Right ankle pitch angle')
% set(gca,'fontsize',fs)
% subplot(5,2,7)
% plot(t,anglesIm(7,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(7,:),'g','LineWidth',2)
% grid on
% axis tight
% ylabel('angle [rad]')
% title('Left hip roll angle')
% set(gca,'fontsize',fs)
% subplot(5,2,8)
% plot(t,anglesIm(8,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(8,:),'g','LineWidth',2)
% grid on
% axis tight
% ylabel('angle [rad]')
% title('Right hip roll angle')
% set(gca,'fontsize',fs)
% subplot(5,2,9)
% plot(t,anglesIm(9,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(9,:),'g','LineWidth',2)
% grid on
% axis tight
% xlabel('time [s]')
% ylabel('angle [rad]')
% title('Left ankle roll angle')
% set(gca,'fontsize',fs)
% subplot(5,2,10)
% plot(t,anglesIm(10,:),'b--','LineWidth',2)
% hold on
% plot(t,anglesImt(10,:),'g','LineWidth',2)
% grid on
% axis tight
% xlabel('time [s]')
% ylabel('angle [rad]')
% title('Right ankle roll angle')
% set(gca,'fontsize',fs)
% l1 = legend('Motions with imitation learning','Motions after reinforcement learning','Location',[0.5 0.95 0.05 0.05],'Orientation','horizontal');
% set(l1,'fontsize',fslegend)