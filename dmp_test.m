%%  Import trajectory

% take simple sine function for example
dt = 0.1;
t = 0:dt:2*pi;
y = 10*sin(t);
dy = 10*cos(t);
ddy = -10*sin(t);
y0 = y(1);
yg = y(end);

%%
N_dmp = 1;      % number of dmps, number of curves to be imitated 
N_bf = 1000;     % number of basic activation function
weights = zeros(N_dmp,N_bf);
ay = 25;    % ay & by: constants of the second order system
by = ay/4;

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


% Forcing term
wpsi = w*psi;
f = wpsi./(sum(psi)).*x*(yg-y0);

% Reproducing
ddyIm = zeros(1,length(t));
dyIm = zeros(1,length(t));
yIm = zeros(1,length(t));
yIm(1) = y0;

for j = 2:length(t)
%     if j == 1
%         ddyIm(j) = 1/tau^2*(ay*(by*(yg-yIm(j))-tau*dyIm(j))+f(j));
%         dyIm(j) = dyIm(j)+ddyIm(j)*dt;
%         yIm(j) = yIm(j)+dyIm(j)*dt;
%     else
        ddyIm(j-1) = 1/tau^2*(ay*(by*(yg-yIm(j-1))-tau*dyIm(j-1))+f(j-1));
        dyIm(j) = dyIm(j-1)+ddyIm(j-1)*dt;
        yIm(j) = yIm(j-1)+dyIm(j-1)*dt;
%     end    
end
figure(1)
plot(t,yIm,'k','LineWidth',2)
hold on
plot(t,y,'r','LineWidth',2)
pause(3)
%% Time scaling
tau1 = [1 2 5 0.8 0.5 0.1];
for i = 1:1
    tau11 = 2;
    t1 = 0:dt:tau11*t(end);
    x1 = exp(-ax/tau11*t1);

    des_c1 = linspace(0,max(t1),N_bf);    % point in time where to put the activation function
    c1 = exp(-ax/tau11*des_c1);             % point in x where to put the activation function
    h1 = N_bf^1.5*ones(1,N_bf)./c1;       % variance of each activation function, trial and error

    psi1 = zeros(N_bf,length(x1));
    for k = 1:N_bf
        for l = 1:length(x1)
            psi1(k,l) = exp(-h1(k)*(x1(l)-c1(k))^2);
        end
    end 
    wpsi1 = w*psi1;
    f1 = wpsi1./(sum(psi1)).*x1*(yg-y0);

    % Reproducing
    ddyIm1 = zeros(1,length(0:dt:tau11*t(end)));
    dyIm1 = zeros(1,length(0:dt:tau11*t(end)));
    yIm1 = zeros(1,length(0:dt:tau11*t(end)));
%     yIm1(1) = y0;

    for k = 2:length(t1)
%         if k == 1
%             ddyIm1(k) = 1/tau11^2*(ay*(by*(yg-yIm1(k))-tau11*dyIm1(k))+f1(k));
%             dyIm1(k) = dyIm1(k)+ddyIm1(k)*dt;
%             yIm1(k) = y0;
%         else
            ddyIm1(k-1) = 1/tau11^2*(ay*(by*(yg-yIm1(k-1))-tau11*dyIm1(k-1))+f1(k-1));
            dyIm1(k) = dyIm1(k-1)+ddyIm1(k-1)*dt;
            yIm1(k) = yIm1(k-1)+dyIm1(k-1)*dt;
%         end    
    end
    figure(1)
    plot(t1,yIm1,'LineWidth',2)
    hold on
    pause(2)
end


%% Root locus analysis
% close all;
% 
% ay = 25;
% by = ay/4;
% tau1 = 0.2;
% num1 = 1/tau1^2*ay*by;
% den1 = [1 1/tau1*ay 1/tau1^2*ay*by];
% num2 = [0.01*exp(-25*0.01/(2*0.1)) 0];
% den2 = [1 2*exp(-25*0.01/(2*0.1)) exp(-2*25*0.01/0.1)];
% sysc1 = tf(num1,den1)
% sysd1 = c2d(tf(num1,den1),0.01)
% figure(2)
% rlocus(sysd1)
% bode(sysc1)
syms tau T
A = 25; B = 25/4;
% T = 0.01;
% a = [1-A*B*T^2/(2*tau^2) T-A*T^2/(2*tau);-A*B*T/tau^2 1-A*T/tau]
a = [1 T;-T/tau^2*A*B 1-T/tau*A]
eig(a)

%%
% figure(1)
% subplot(2,2,1)
% plot(t,f,'r','LineWidth',2)
% grid on
% axis([0,2*pi,-2000,2000])
% xlabel('time [s]')
% title('Forcing term with \tau = 1')
% set(gca,'fontsize',20)
% subplot(2,2,2)
% plot(t1,f1,'b','LineWidth',2)
% grid on
% axis([0,4*pi,-2000,2000])
% xlabel('time [s]')
% title('Forcing term with \tau = 2')
% set(gca,'fontsize',20)
% subplot(2,2,3)
% plot(t,yImuf,'r','LineWidth',2)
% grid on
% axis([0,2*pi,-0.04,0])
% xlabel('time [s]')
% title('Unforced second order system with \tau = 1')
% set(gca,'fontsize',20)
% subplot(2,2,4)
% plot(t1,yImuf1,'b','LineWidth',2)
% grid on
% axis([0,4*pi,-0.04,0])
% xlabel('time [s]')
% title('Unforced second order system with \tau = 2')
% set(gca,'fontsize',20)
% figure(2)
% 
% plot(t,y,'k','LineWidth',4)
% hold on
% plot(t,yIm,'r','LineWidth',2)
% hold on
% plot(t1,yIm1,'b','LineWidth',2)
% grid on
% xlabel('time [s]')
% axis([0,4*pi,-11,11])
% 
% l1 = legend('Original curve','Without temporal scaling: \tau = 1','With temporal scaling: \tau = 2');
% set(gca,'fontsize',20)
% set(l1,'fontsize',15)
