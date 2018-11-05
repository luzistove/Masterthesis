% This file defines the esssential geometric parameters as well as the masses
% and the inertia of the NAO robot. All the length values are with unit m; 
% mass values are with unit kg and inertia values with unit kg*m2.
% For details please refer to the official website. 
% The [X,Y,Z] coordinate is defined as: the X axis points at the forward 
% direction of the walking; the Z axis points upwards and these three axes 
% follow the right-hand rule.
%% Geometric parameters
foot_height = 0.02;
foot_length = 0.1;
foot_width = 0.05;
leg_radius = 0.01;
tibia_length = 0.1029;
thigh_length = 0.1;
hip_off_Z = 0.085;
hip_off_Y = 0.05;
neck_off_Z = 0.1265;
shoulder_off_Y = 0.098;
lowerarm_length = 0.05595;
upperarm_length = 0.105;
arm_radius = leg_radius;
hand_length = 0.05775;
elbow_off_Y = 0.015;
neck_length = 0.03;
neck_radius = 0.02;
ground_length = 5;
ground_width = 2;
ground_height = 0.0025;
contact_radius = 1e-5;
hip_radius = leg_radius;

torso_height = foot_height + tibia_length + thigh_length + 0.5*hip_off_Z;

%% Masses
foot_mass = 0.17184;
ankle_mass = 0.13416;
tibia_mass = 0.30142;
thigh_mass = 0.38968;
hip_mass = 0.14053;
pelvis_mass = 0.06981;
hand_mass = 0.18533;
lowerarm_mass = 0.07761;
elbow_mass = 0.06483;
upperarm_mass = 0.15777;
shoulder_mass = 0.09304;
neck_mass = 0.07842;
torso_mass = 1.0496;
head_mass = 0.60533;
robot_mass = 5.305;

%% Center of Mass
% torso_com = [-0.00413 0 0.04342];
% neck_com = [-1e-5 0 -0.02742];
% head_com = [-0.00112 0 0.05258];
% 
% shoulder_r_com = [-0.00165 0.02663 0.00014];
% shoulder_l_com = [-0.00165 -0.02663 0.00014];
% 
% upperarm_r_com = [0.02455 -0.00563 0.0033];
% upperarm_l_com = [0.02455 0.00563 0.0033];
% 
% lowerarm_r_com = [0.02665 -0.00281 0.00076];
% lowerarm_l_com = [0.02665 0.00281 0.00076];
% 
% hand_r_com = [0.04343 0.00088 0.00308];
% hand_l_com = [0.04343 -0.00088 0.00308];
% 
% hip_r_com = [-0.01549 -0.0029 -0.00515];
% hip_l_com = [-0.01549 0.0029 -0.00515];
% 
% thigh_r_com = [0.00138 -0.00221 -0.05373];
% thigh_l_com = [0.00138 0.00221 -0.05373];
% 
% tibia_r_com = [0.00453 -0.00225 -0.04936];
% tibia_l_com = [0.00453 0.00225 -0.04936];
% 
% foot_r_com = [0.02542 -0.0033 -0.03239];
% foot_l_com = [0.02542 0.0033 -0.03239];



%% Inertia
foot_inertia_l_m = [0.000269 0.000644 0.000526];
foot_inertia_l_p = [1.87409e-5 0.000139 -5.6957e-6];

foot_inertia_r_m = [0.000269 0.000643 0.000526];
foot_inertia_r_p = [-1.8849e-5 0.000139  5.87505e-6];

ankle_inertia_l_m = [3.8600e-5 7.4265e-5 5.4865e-5];
ankle_inertia_l_p = [1.8340e-8  3.8619e-6 -2.634e-8];
               
ankle_inertia_r_m = [3.8508e-5 7.4311e-5 5.4913e-5];  
ankle_inertia_r_p = [-4.580e-9 3.8746e-6 6.4340e-8]; 

tibia_inertia_l_m = [0.00118 0.00113 0.00019];
tibia_inertia_l_p = [3.9495e-5  3.6497e-5 6.3362e-7];
               
tibia_inertia_r_m = [0.00118 0.00113 0.00019];
tibia_inertia_r_p = [-3.8476e-5 2.7997e-5 -8.965e-7];

thigh_inertia_l_m = [0.00164 0.00159 0.00030];
thigh_inertia_l_p = [3.8362e-5 8.5307e-5 9.2451e-7];  
               
thigh_inertia_r_m = [0.00164 0.00159 0.00030]; 
thigh_inertia_r_p = [-3.9176e-5 8.5883e-5 -8.3954e-7]; 

hip_inertia_l_m = [2.7584e-5 9.8271e-5 8.8099e-5];
hip_inertia_l_p = [-4.1900e-9 -4.0816e-6 -2.2330e-8];
             
hip_inertia_r_m = [2.7586e-5 9.8270e-5 8.8103e-5];
hip_inertia_r_p = [2.5100e-9 -4.1082e-6  1.9190e-8];
             
pelvis_inertia_l_m = [8.1502e-5 0.00010 6.2624e-5];
pelvis_inertia_l_p = [2.3455e-5 1.2748e-5 -4.9945e-6];

pelvis_inertia_r_m = [8.9972e-5 0.00011 6.6887e-5];
pelvis_inertia_r_p = [-2.7701e-5 1.2735e-5 5.0022e-6];
                
hand_inertia_l_m = [7.0549e-5 0.00036 0.00035];
hand_inertia_l_p = [3.1777e-6 -2.2474e-5 5.7160e-6];
              
hand_inertia_r_m = [7.0549e-5 0.00036 0.00035];
hand_inertia_r_p = [3.1777e-6 -2.2474e-5 5.7160e-6];
              
lowerarm_inertia_l_m = [2.5332e-5 8.9132e-5 8.7287e-5];
lowerarm_inertia_l_p = [-2.6550e-8 7.4890e-8 -2.3427e-6];
                  
lowerarm_inertia_r_m = [2.5391e-5 8.9220e-5 8.7248e-5];
lowerarm_inertia_r_p = [2.6940e-8 -6.0117e-8 2.3324e-6];
                  
elbow_inertia_l_m = [5.5971e-6 7.5433e-5 7.64443e-5];
elbow_inertia_l_p = [-1.8400e-9 4.3190e-8 4.2100e-9];
               
elbow_inertia_r_m = [5.5971e-6 7.5433e-5 7.64443e-5];
elbow_inertia_r_p = [-1.8400e-9 4.3190e-8 4.2100e-9];  

upperarm_inertia_l_m = [9.3900e-5 0.00037 0.00034];
upperarm_inertia_l_p = [-2.4598e-6 -2.6995e-5 -4.7144e-5];
                  
upperarm_inertia_r_m = [0.00011 0.00037 0.00035];
upperarm_inertia_r_p = [1.2098e-5 -2.6046e-5 7.6691e-5];
                  
shoulder_inertia_l_m = [8.4284e-5 1.4156e-5 8.6419e-5];
shoulder_inertia_l_p = [-1.9720e-8 2.3380e-8 -2.0820e-6];

shoulder_inertia_r_m = [8.4284e-5 1.4156e-5 8.6419e-5];
shoulder_inertia_r_p = [1.9720e-8 2.3380e-8 2.0820e-6];
                  
head_inertia_m = [0.00263 0.00249 0.00099];
head_inertia_p = [-2.9960e-5 4.0985e-5 8.7881e-6];
         
neck_inertia_m = [7.4993e-5 7.6000e-5 5.5337e-6];
neck_inertia_p = [-5.2950e-8  1.8340e-8 1.5700e-9];           
            
torso_inertia_m = [0.00506 0.00488 0.00161];
torso_inertia_p = [-2.7079e-5 0.00015 1.4312e-5];

%% Other parameters
g = 9.81;
% gait_period = 1.0;
%time = linspace(0,gait_period,7)';
% ankle_motion = deg2rad([-7.5 10 10 5 0 -10 -7.5]');
% knee_motion = deg2rad([10, -5, 2.5, -10, -10, 15, 10]');
% hip_motion = deg2rad([-10, -7.5, -15, 10, 15, 10, -10]');
% contact_stiffness = 22 * robot_mass * 9.81 / 8 / 0.0001;
contact_stiffness = (2*foot_mass+2*ankle_mass+2*tibia_mass+2*thigh_mass+2*hip_mass+...
    torso_mass)* 9.81/ (1e-5);
%contact_stiffness = 1e10;
contact_damping = 0;
kinetic_friction = 1000;
static_friction = 1000;

%% cart-table model
g = 9.81;
A = [0 1 0;0 0 1;0 0 0];
B = [0 0 1]';
C = [1 0 -torso_height/g];
D = 0;
Ts = 0.01;
% t = 0:0.01:15;
sys_ZMP = ss(A,B,C,D);
sys_ZMPd = c2d(sys_ZMP,0.01,'zoh');
[Ad,Bd,Cd,Dd] = dssdata(sys_ZMPd);
% figure(1)
% step(sys_ZMP)
% figure(2)
% step(sys_ZMPd)

%%
% kf = [0 1 2 3 4];
% ks = [0 1 2 3 4];
% speed = [0.40 2.3 2.8 2.8 2.80;0.90 3.8 7.7 7.67 7.67;1.02 8.7 9.3 9.3 9.3;...
%     1.03 9.2 9.5 9.55 9.55;1.03 9.2 9.3 9.50 9.50];
% [x,y] = meshgrid(0:4);
% mesh(x,y,speed)
% colorbar
% xlabel('log_{10} \mu_k')
% ylabel('log_{10} \mu_k')
% zlabel('Speed (cm/s)')


% figure(1)
% r = 0.05;
% xc = 3.3;
% yc = 2.75;
% theta = linspace(0,2*pi);
% x = r*cos(theta) + xc;
% y = r*sin(theta) + yc;
% h1 = plot(x,y,'r','LineWidth',2);
% hold on
% 
% x1=1.8;
% x2=2.2;
% y1=2.4;
% y2=2.8;
% x = [x1, x2, x2, x1, x1];
% y = [y1, y1, y2, y2, y1];
% h2 = plot(x, y, 'b--', 'LineWidth', 1.5);
% hold on
% 
% x1=2.3;
% x2=2.7;
% y1=3.1;
% y2=3.5;
% x = [x1, x2, x2, x1, x1];
% y = [y1, y1, y2, y2, y1];
% h3 = plot(x, y, 'b--', 'LineWidth', 1.5);
% hold on
% 
% x1=2.8;
% x2=3.2;
% y1=3.6;
% y2=4;
% x = [x1, x2, x2, x1, x1];
% y = [y1, y1, y2, y2, y1];
% h4 = plot(x, y, 'b--', 'LineWidth', 1.5);
% hold on
% 
% x1=3.6;
% x2=4;
% y1=3.2;
% y2=3.6;
% x = [x1, x2, x2, x1, x1];
% y = [y1, y1, y2, y2, y1];
% h5 = plot(x, y, 'b--', 'LineWidth', 1.5);
% hold on
% 
% x1=0.1;
% x2=0.3;
% y1=3.35;
% y2=3.45;
% x = [x1, x2, x2, x1, x1];
% y = [y1, y1, y2, y2, y1];
% h6 = plot(x, y, 'k', 'LineWidth', 1.5);
% hold on
% 
% x1=0.1;
% x2=0.3;
% y1=3.55;
% y2=3.65;
% x = [x1, x2, x2, x1, x1];
% y = [y1, y1, y2, y2, y1];
% h7 = plot(x, y, 'k', 'LineWidth', 1.5);
% hold on
% 
% x = [0.2 1 1.5 2 2.5 3 3.2];
% y = [3.5 3 2.3 2 2 2.5 2.75];
% xx = 0.2:0.1:3.2;
% h8 = plot(xx,pchip(x,y,xx),'g-->','LineWidth',1);
% grid on
% legend([h1,h2,h6,h8],'Target','Obstacles','Robot original position','Planned path')
% xlabel('x[m]')
% ylabel('y[m]')
% axis([0,5,0,5])
% axis equal