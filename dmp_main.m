t = 0:0.01:10;
dt = 0.01;
%%  Import trajectory

% take simple sine function for example
% y = sin(t);
% dy = cos(t);
% ddy = -sin(t);
% y0 = y(1);
% yg = y(end);

y = 2*t;
dy = 2*ones(1,length(t));
ddy = zeros(1,length(t));
y0 = y(1);
yg = y(end);


% check offset
% check if the initial position and the goal are the same, if so, give
% slight offset so that the forcing term is never 0
if y0 == yg
    yg = yg+0.0001;
end

%% Canonical system
ax = 1;
tau = 1;
x = exp(-ax/tau*t);
% figure(1)
% plot(t,x)

%% Generating activation function
N_dmp = 1;  % number of dmps, number of curves to be imitated 
N_bf = 100;  % number of basic activation function

des_c = linspace(0,max(t),N_bf);    % point in time where to put the activation function
c = exp(-ax/tau*des_c);                 % point in x where to put the activation function
h = N_bf^1.5*ones(1,N_bf)./c;   % variance of each activation function, trial and error


psi = zeros(N_bf,length(x));
for i = 1:N_bf
    for j = 1:length(x)
        psi(i,j) = exp(-h(i)*(x(j)-c(i))^2);
    end
end

% figure(2)
% plot(x,psi)
%% Forcing term
% Calculating weights
ay = 25;    % ay & by: constants of the second order system
by = ay/4;
fT = ddy-ay*(by*(yg-y)-dy);     % f_target

w = zeros(1,N_bf);  % weights in the forcing term
ksi = (yg-y0)*x;
for i = 1:N_bf
    w(i) = (ksi*diag(psi(i,:))*fT')/(ksi*diag(psi(i,:))*ksi');
end

% forcing term
wpsi = w*psi;
f = wpsi./(sum(psi)).*x*(yg-y0);

%% DMP imitation
% 
ddy_im = zeros(1,length(t));
dy_im = zeros(1,length(t));
y_im = zeros(1,length(t));
y_im(1) = y0;

for i = 1:length(t)
    if i == 1
        ddy_im(i) = (ay*(by*(yg-y_im(i))-dy_im(i))+f(i))/tau;
        dy_im(i) = dy_im(i)+ddy_im(i)*dt;
        y_im(i) = y_im(i)+dy_im(i)*dt;
    else
        ddy_im(i) = (ay*(by*(yg-y_im(i-1))-dy_im(i-1))+f(i))/tau;
        dy_im(i) = dy_im(i-1)+ddy_im(i)*dt;
        y_im(i) = y_im(i-1)+dy_im(i)*dt;
    end    
    
end
figure(3)

plot(t,y_im,'b--')
hold on
plot(t,y,'r')