x = [1 2 3];
x_t = {'path_tracking', 'soldering', 'suturing'};

y_L   = [0.6 0.64 0.68 0.57 0.55 0.3 0.1 0.0 0.2 0.4 0.35];
y_L_0 = [0.6 0.64 0.68 0.58 0.55 0.5 0.4 0.6 0.6 0.5 0.5];

old_time        = linspace(1,200,11);
new_t           = 1:1:200;
[x_L,qd,qdd,tt]   = TrajectoryGeneration(old_time,y_L',new_t);
[x_L_0,qd,qdd,tt]   = TrajectoryGeneration(old_time,y_L_0',new_t);

new_n           = size(q,1);

close all
% Create figure
figure1 = figure('Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');


plot(x,y_o(1:3),'r')
hold on
plot(x,y_p(1:3),'g')
plot(x,y_optimal_o(1:3),'b')
plot(x,y_optimal_p(1:3),'k')

% Create figure
figure1 = figure('Color',[1 1 1]);