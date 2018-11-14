close all;
clear all;

%% Weights of imitation learning
load('weights')

% These weights are treated as initialized weights in RL 
%% DMPs parameters

t = 0:0.01:12-0.01;
dt = 0.01;

N_dmp = size(weights,1);
N_bf = size(weights,2);
anglesIm = zeros(N_dmp,length(t));

ddyIm = zeros(1,length(t));
dyIm = zeros(1,length(t));
yIm = zeros(1,length(t));

ay = 25;    % ay & by: constants of the second order system
by = ay/4;

ax = 1;
tau = 10;

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

%% RL process
% Reinforcement learning is a learning process of many rollouts. 1 rollout is 1 time weights update, 1 time DMP reproduction 
% with DMPs, 1 time simulation and 1 evaluation
maxIt = 1000;   % max number of rollouts. 
sig = 100;
mu = 0;
count = 1;

R = zeros(1,maxIt);     % Rewards

while count <= maxIt
    weights = weights + normrand(mu,sig,[N_dmp,N_bf]);
    
    for i = 1:N_dmp
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
    
        % Forcing term
        wpsi = weights(1,:)*psi;
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
    
    sim('dmpRLtest.slx')    % run simulation and get results
    
end


