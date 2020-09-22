%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                4D Data Visualization                 %
%              with MATLAB Implementation              %
%                                                      %
% Author: M.Sc. Eng. Hristo Zhivomirov        02/16/13 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all

% form the axes
x = -1:0.1:1;   % first dimension independent variable
y = -1:0.1:1;   % second dimension independent variable
z = -1:0.1:1;   % third dimension independent variable
[X, Y, Z] = meshgrid(x, y, z);	% form the 3D grid

% write the equation that describes the fourth dimension
data = abs(cos(X) + cos(Y) + cos(Z)); 

% plot the data
figure(1)
subplot(2, 2, 1)
slice(X, Y, Z, data, 0, 0, 0);
shading interp
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('X')
ylabel('Y')
zlabel('Z')
alpha(0.75) 
colorbar

% plot the data (cont.)
subplot(2, 2, 2)
slice(X, Y, Z, data, [-1 -0.5 0 0.5 1], [], []);
shading interp
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('X')
ylabel('Y')
zlabel('Z')
alpha(0.75) 
colorbar

% plot the data (cont.)
subplot(2, 2, 3)
slice(X, Y, Z, data, [], [-1 -0.5 0 0.5 1], []);
shading interp
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('X')
ylabel('Y')
zlabel('Z')
alpha(0.75) 
colorbar

% plot the data (cont.)
subplot(2, 2, 4)
slice(X, Y, Z, data, [], [], [-1 -0.5 0 0.5 1]);
shading interp
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('X')
ylabel('Y')
zlabel('Z')
alpha(0.75) 
colorbar