%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                4D Data Visualization                 %
%              with MATLAB Implementation              %
%                                                      %
% Author: M.Sc. Eng. Hristo Zhivomirov        02/16/13 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all

% form the axes
x = 0:0.5:10;   % first dimension independent variable
y = 0:0.5:10;   % second dimension independent variable
z = 0:0.5:10;   % third dimension independent variable
[X, Y, Z] = meshgrid(x, y, z);  % form the 3D grid

% write the equation that describes the fourth dimension
data = abs(cos(X) + cos(Y) + cos(Z));   

% plot the data
figure(1)
scatter3(X(:), Y(:), Z(:), 25, data(:), 'filled')
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('X')
ylabel('Y')
zlabel('Z')
colorbar
