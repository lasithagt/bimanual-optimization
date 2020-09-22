x = 2*rand(2500,1) - 1; 
y = 2*rand(2500,1) - 1; 
z = 2*rand(2500,1) - 1;
v = x.^2 + y.^3 - z.^4;

d = -1:0.05:1;
[xq,yq,zq] = meshgrid(d,d,0.2);

vq = griddata(x,y,z,v,xq,yq,zq);

% plot3(x,y,v,'ro')
% hold on
mesh(xq,yq,zq,vq,'FaceAlpha','0.5', 'FaceColor', 'flat', 'EdgeColor' ,'none')

% surf(xq,yq,0,vq)