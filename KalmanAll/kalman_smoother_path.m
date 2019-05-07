% Make a point move in the 3D plane
% State = (x y xdot ydot). We only observe (x y).

% X(t+1) = F X(t) + noise(Q)
% Y(t) = H X(t) + noise(R)
addpath ./Kalman
addpath ./KPMstats
addpath ./KPMtools

close all;
clear

%% Generate data for sinusoidal. For sanity testing.
t   = linspace(0, pi, 100);
x_  = sin(t);
dt_ = 1/100;
F_  = [1 1;0 1]; 
H_  = [1 0];

ss_ = 2; os_ = 1;

Q_  = 1*eye(ss_);
R_  = 0.01*eye(os_);

initx_ = [x_(1) 0]';
initV_ = 10*eye(ss_);

[xfilt, Vfilt, VVfilt, loglik] = kalman_filter(x_, F_, H_, Q_, R_, initx_, initV_);
dfilt    = x_ - xfilt(1,:);
mse_filt = sqrt(sum(sum(dfilt.^2)));

% figure(1)
% clf
% plot(x_);
% hold on
% plot(xfilt(1,:));
% 
% ylabel('x')
% legend('true', 'filtered')
% 
% figure(2)
% plot(xfilt(2,:));


%%
ss = 9; % state size
os = 3; % observation size
dt = 1; 

%
F = [1 dt 0 0 0 0 0 0 0;0 1 dt 0 0 0 0 0 0;0 0 1 0 0 0 0 0 0;...
     0 0 0 1 dt 0 0 0 0;0 0 0 0 1 dt 0 0 0;0 0 0 0 0 1 0 0 0;...
     0 0 0 0 0 0 1 dt 0;0 0 0 0 0 0 0 1 dt;0 0 0 0 0 0 0 0 1]; 
H = [1 0 0 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0;0 0 0 0 0 0 1 0 0];

Q      = 0.0001*eye(ss);
Q(3,3) = 2;
Q(6,6) = 2;
Q(9,9) = 2;
R      = 0.1*eye(os);

% Load some data to smoothen
load('IDEAS 2014 Data/Subject 1/Subject1Test1.mat')
dataXYZ = motiondata.instrument(1).data.pos; % To test

initx = [dataXYZ(1,1) 0 0 dataXYZ(1,2) 0 0 dataXYZ(1,3) 0 0]';
initV = 10*eye(ss);

% Get x,y,z from sample data.
x = dataXYZ(:,1);  
y = dataXYZ(:,2);
z = dataXYZ(:,3);

%
[xfilt, Vfilt, VVfilt, loglik] = kalman_filter(dataXYZ', F, H, Q, R, initx, initV);
[xsmooth, Vsmooth]             = kalman_smoother(dataXYZ', F, H, Q, R, initx, initV);

%
dfilt      = x' - xfilt(1,:);
mse_filt   = sqrt(sum(sum(dfilt.^2)))
% 
dsmooth    = x' - xsmooth(1,:);
mse_smooth = sqrt(sum(sum(dsmooth.^2)))
% 
% 
figure(3)
clf
hold on
plot(x(100:200), y(100:200), 'ks-');
% plot(y(1,:), y(2,:), 'g*');
plot(xfilt(1,100:200), xfilt(4,100:200), 'rx:');
T = 100;
for t=1:T, plotgauss2d(xfilt([1 4],t+100), Vfilt([1 4], [1 4], t+100)); end
hold off

xlabel('x')
ylabel('y')
legend('true', 'filtered')

figure(4)
vel = diff(x);
plot(xfilt(2,:),'k');
hold on
plot(vel,'r')
hold off
% % 

figure(5)
hold on
plot(x(100:200), y(100:200), 'ks-');
% plot(y(1,:), y(2,:), 'g*');
plot(xsmooth(1,100:200), xsmooth(4,100:200), 'rx:');
T = 100;
for t=1:T, plotgauss2d(xsmooth([1 4],t+100), Vsmooth([1 4], [1 4], t+100)); end
hold off

xlabel('x')
ylabel('y')
legend('true', 'smoothed')

figure(6)
vel = diff(x);
plot(xsmooth(2,:),'k');
hold on
plot(vel,'r')
hold off

figure(7)
% vel = diff(x);
plot(xsmooth(3,:),'k');
hold on
% plot(vel,'r')
