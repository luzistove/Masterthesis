
clear all;
%% RL process
% Reinforcement learning is a learning process of many rollouts. 1 rollout is 1 time weights update, 1 time DMP reproduction 
% with DMPs, 1 time simulation and 1 evaluation
N_dmp = 10;
N_dmp1 = 2;
N_bf = 50;
load('E:\TUHH master\Master thesis\Code\dmp\primiRL\wSideRL.mat');
load('E:\TUHH master\Master thesis\Code\dmp\primiRL\primiSideRL.mat');
ax = 1;
tau = 1;
ay = 25; by = ay/4;
dt = 0.01;
t = 0:0.01:0.01*(size(primiSideRL,2)-1);

maxIt = 100;   % max number of rollouts. 
sig = 100;
mu = 0;
count = 1;
cap = 100;   % capacity of the importance sampler
R = zeros(1,cap);     % Rewards
rollout = zeros(N_bf,cap*N_dmp1);
explore = zeros(N_bf,cap*N_dmp1);
tol = 1e-3;
wSideRLold = zeros(N_dmp1,N_bf);
%% RL process
run('Nao_parameter')
while (norm(wSideRL(1,:)-wSideRLold(1,:)) >= tol && norm(wSideRL(2,:)-wSideRLold(2,:)) >= tol)%count <= maxIt%3
    temp = 2*normrnd(mu,sig,[N_dmp1,N_bf]);
    wSideRL(1:2,:) = wSideRL(1:2,:) + temp;

    anglesIm = zeros(N_dmp,size(primiSideRL,2));
    for i = 1:N_dmp
        y = primiSideRL(i,:);
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
    
        wpsi = wSideRL(i,:)*psi;
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
        reFall = 0;
    else
        reFall = 100;
    end
    
    % Whether still forward or backward offset
    if abs((position(end,2)-position(1,2))) < 0.05
        reOff = 100;
    else
        reOff = -1000;
    end
    
    % Whether angle is beyond limit
    for i = 1:10
        switch i
            case 1
                if (min(anglesIm(i,:)) <= deg2rad(-40) || max(anglesIm(i,:)) >= deg2rad(15))
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 2
                if (min(anglesIm(i,:)) <= rad2deg(-40) || max(anglesIm(i,:)) >= rad2deg(15))
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 3
                if (min(anglesIm(i,:)) <= -0.3 || max(anglesIm(i,:)) > 0)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 4
                if (min(anglesIm(i,:)) <= -0.3 || max(anglesIm(i,:)) > 0)
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
                if (min(anglesIm(i,:)) < -0.2 || max(anglesIm(i,:)) > 0)
                    angBeyond = 1;
                    break;
                else
                    angBeyond = 0;
                end
            case 8
                if (min(anglesIm(i,:)) < 0 || max(anglesIm(i,:)) > 0.2)
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
        reAng = 100;
    end
    % Speed
%     reSpeed = abs((position(1,3)-position(end,3)))*(100);
    if (position(1,3)-position(end,3)) > 25
        reSpeed = 100;
    else 
        reSpeed = -1000;
    end
    % Direction
    if position(end,3) > position(1,3)
        reDirection = 0;
    else 
        reDirection = 10;
    end
    
    % simulation end by error
    if stop(end) == 1
        reEnd = -100;
    else
        reEnd = 0;
    end
    
    reAll = reFall+reOff+reAng+reEnd;%+reSpeed;%+reDirection;
    
    if count <= cap
        R(1,count) = reAll;
        rollout(:,(count-1)*2+1:(count-1)*2+2) = wSideRL(1:2,:)';
        explore(:,(count-1)*2+1:(count-1)*2+2) = temp';
%         for k = 1:count
%             if k >= 2
%                 
%             end
%         end
    end
    
    for i = 1:cap
        if reAll > R(1,i)
            for j = cap:-1:i+1
                R(1,j) = R(1,j-1);
                rollout(:,(j-1)*2+1:(j-1)*2+2) = rollout(:,(j-2)*2+1:(j-2)*2+2);
                explore(:,(j-1)*2+1:(j-1)*2+2) = explore(:,(j-2)*2+1:(j-2)*2+2);
            end      
%             R(1,i) = reAll;
%             rollout(:,i*10+1:i*10+10) = wSideRL';
        break;

        end
    end
    R(1,i) = reAll;
    rollout(:,(i-1)*2+1:(i-1)*2+2) = wSideRL(1:2,:)';
%     maxR = max(R);
%     for j = 1:cap
%         if R(1,j) == maxR
%             break;
%         else
% 
%         end
%     end
%         
%     maxIndex = j;
%     if reAll >= maxR
%        for k = cap:-1:maxIndex+1
%             R(1,k) = R(1,k-1);
%             rollout(:,k*10+1:k*10+10) = rollout(:,(k-1)*10+1:(k-1)*10+10);
%        end
%        R(1,i) = reAll;
%        rollout(:,i*10+1:i*10+10) = wSideRL'; 
%     end
    
    
    
    num = zeros(N_bf,N_dmp1);
    den = zeros(N_bf,N_dmp1);
    
    for i=1:cap
%         num = num+((rollout(:,(i-1)*10+1:(i-1)*10+10)-wSideRL').*R(1,i));
%         den = den+R(1,i);
        num = num+(explore(:,(i-1)*2+1:(i-1)*2+2).*R(1,i));
        den = den+R(1,i);
    end
    % update   
    wSideRLold = wSideRL(1:2,:);
    wSideRL(1:2,:) = wSideRL(1:2,:)+(num./(den+1.e-5))';
%     figure(2)
%     plot(wSideRL(:,8))
    count = count+1;
    
    
end


