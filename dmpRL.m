
% clear all;
%% RL process
% Reinforcement learning is a learning process of many rollouts. 1 rollout is 1 time weights update, 1 time DMP reproduction 
% with DMPs, 1 time simulation and 1 evaluation
N_dmp = 10;
N_dmp1 = 4;
N_bf = 50;
load('E:\TUHH master\Master thesis\Code\dmp\primiRL\wStraightRL.mat');
load('E:\TUHH master\Master thesis\Code\dmp\primiRL\primiStraightRL.mat');
ax = 1;
tau = 1;
ay = 25; by = ay/4;
dt = 0.01;
t = 0:0.01:0.01*(size(primiStraightRL,2)-1);

maxIt = 3;   % max number of rollouts. 
sig = 100;
mu = 0;
count = 1;
cap = 10;   % capacity of the importance sampler
R = ones(1,cap);     % Rewards
rollout = zeros(N_bf,cap*N_dmp1);
explore = zeros(N_bf,cap*N_dmp1);
tol = 1e-5;
wStraightRLold = zeros(N_dmp1,N_bf);

p1 = 1; p2 = p1+N_dmp1-1;
%% RL process
% wStraightRL = wStraightRL;
run('Nao_parameter')
while count < 2%%3(norm(wStraightRL(p1:p2,:)-wStraightRLold) >= tol )%
    temp = normrnd(mu,sig,[N_dmp1,N_bf]);
    
    wStraightRL(p1:p2,:) = wStraightRL(p1:p2,:);% + temp;

    anglesIm = zeros(N_dmp,size(primiStraightRL,2));
    for i = 1:N_dmp
        y = primiStraightRL(i,:);
        y0 = y(1); yg = y(end);
    
    
        % check if the initial position and the goal are the same, if so, give
        % slight offset so that the forcing term is never 0
        if y0 == yg
            yg = yg+0.001;
        end

    
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

        
        wpsi = wStraightRL(i,:)*psi;
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
    hipPitchL = anglesIm(1,:);
    hipPitchR = anglesIm(2,:);
    kneeBendL = anglesIm(3,:);
    kneeBendR = anglesIm(4,:);
    anklePitchL = -anglesIm(1,:)+anglesIm(3,:);
    anklePitchR = -anglesIm(2,:)+anglesIm(4,:);
    hipRollL = anglesIm(7,:);
    hipRollR = anglesIm(8,:);
    ankleRollL = -anglesIm(7,:);
    ankleRollR = -anglesIm(8,:);
%     figure(1)
%     plot(anglesIm(8,:))
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
    if abs((position(end,2)-position(1,2))) < 0.01
        reOff = 0;%20000*abs((position(end,2)-position(1,2)));
    else
        reOff = -2000*abs((position(end,2)-position(1,2)));
    end
    
    % Whether angle is beyond limit
    for i = 1:10
        switch i
            case 1
                if (min(anglesIm(i,:)) <= deg2rad(-50) || max(anglesIm(i,:)) >= deg2rad(50))
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 2
                if (min(anglesIm(i,:)) <= rad2deg(-50) || max(anglesIm(i,:)) >= rad2deg(50))
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 3
                if (min(anglesIm(i,:)) <= -0.6 || max(anglesIm(i,:)) > 0)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 4
                if (min(anglesIm(i,:)) <= -0.6 || max(anglesIm(i,:)) > 0)
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
                if (min(anglesIm(i,:)) < -0.3 || max(anglesIm(i,:)) > 0)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 8
                if (min(anglesIm(i,:)) < 0 || max(anglesIm(i,:)) > 0.3)
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
        reAng = -1000;
    else
        reAng = 0;
    end
    % Speed
%     reSpeed = abs((position(1,3)-position(end,3)))*(100);
%     if (position(1,3)-position(end,3)) > 0.4
        reSpeed = 100*abs((position(1,2)-position(end,2)));
%     else 
%         reSpeed = 0;
%     end
    % Direction
    if position(end,3) > position(1,3)
        reDirection = 0;
    else 
        reDirection = 10;
    end
    
    % simulation end by error
    if stop(end) == 1
        reEnd = -10000;
    else
        reEnd = 0;
    end
    
    reAll = reFall+reSpeed+reEnd+reAng;%+reOff+reDirection;
    
    if count <= cap
        R(1,cap-count+1) = reAll;
%         rollout(:,(count-1)*N_dmp1+1:(count-1)*N_dmp1+N_dmp1) = wStraightRL(p1:p2,:)';
%         explore(:,(count-1)*N_dmp1+1:(count-1)*N_dmp1+N_dmp1) = temp';
        rollout(:,(cap-count)*N_dmp1+1:(cap-count)*N_dmp1+N_dmp1) = wStraightRL(p1:p2,:)';
        explore(:,(cap-count)*N_dmp1+1:(cap-count)*N_dmp1+N_dmp1) = temp';
    else
        for i = 1:cap
            if reAll > R(1,i)
                for j = cap:-1:i+1
                    R(1,j) = R(1,j-1);
                    rollout(:,(j-1)*N_dmp1+1:(j-1)*N_dmp1+N_dmp1) = rollout(:,(j-2)*N_dmp1+1:(j-2)*N_dmp1+N_dmp1);
                    explore(:,(j-1)*N_dmp1+1:(j-1)*N_dmp1+N_dmp1) = explore(:,(j-2)*N_dmp1+1:(j-2)*N_dmp1+N_dmp1);
                end      
    %             R(1,i) = reAll;
    %             rollout(:,i*10+1:i*10+10) = wStraightRL';
                break;

            end
        end
        R(1,i) = reAll;
        rollout(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1) = wStraightRL(p1:p2,:)';
        
    end
    

    
    
    num = zeros(N_bf,N_dmp1);
    den = zeros(N_bf,N_dmp1);
    
    for i=1:cap
        num = num+((rollout(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1)-wStraightRL(p1:p2,:)').*R(1,i));
        den = den+R(:,i);
%         num = num+(explore(:,(i-1)*N_dmp1+1:(i-1)*N_dmp1+N_dmp1).*R(1,i));
%         den = den+R(:,i);
    end
    % update 
    update = (num./(den+1.e-5))';
    wStraightRLold = wStraightRL(p1:p2,:);
    wStraightRL(p1:p2,:) = wStraightRL(p1:p2,:)+update;
%     figure(2)
%     plot(wStraightRL(:,8))
    count = count+1

    
end
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

% plot(1:219,posEnd(1,:),'r','LineWidth',2)
% grid on
% xlabel('numbers of iteration')
% ylabel('position [m]')
% set(gca,'fontsize',20)