x = [1 2 3];
x_t = {'path_tracking', 'soldering', 'suturing'};

y_p = [554, 450, 525];
y_o = [0.2, 0.6, 0.8];

y_p = normalize([y_p y_optimal_p],'norm');
y_o = normalize([y_o, y_optimal_o],'norm');

y_optimal_p = [590, 460, 550];
y_optimal_o = [0.5, 0.64, 0.6];

y_optimal_o = normalize(y_optimal_o,'norm');
y_optimal_p = normalize(y_optimal_p,'norm');

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