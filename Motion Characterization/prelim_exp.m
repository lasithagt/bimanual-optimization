fname = 'apple\';
load('forces\apple.mat')
load('em_calib.mat')
ball = apple;
fv = stlread('apple.stl');

v = VideoWriter('apple3.avi');
open(v)

Fv = fv;
[r, c] = size(fv.vertices(:,:));

s = dir(fname);
s = s(3:end);
[S, i] = sort([s.datenum]);
s = s(i);

d1 = dlmread([fname s(1).name],' ',14,0);
d2 = dlmread([fname s(2).name],' ',14,0);
d3 = dlmread([fname s(3).name],' ',14,0);
d4 = dlmread([fname s(6).name],' ',14,0);
d5 = dlmread([fname, s(4).name],' ',14,0);
d6 = dlmread([fname, s(5).name],' ',14,0);
d7 = dlmread([fname, s(7).name],' ',14,0);
d8 = dlmread([fname, s(8).name],' ',14,0);
D1 = [d1(:,1), -1 .* d1(:,2:3)];
Dr1 = (d1(:,4:6)-[0,0,c1]);
D2 = [d2(:,1), -1 .* d2(:,2:3)];
Dr2 = (d2(:,4:6)-[0,0,c2]);
D3 = [d3(:,1), -1 .* d3(:,2:3)];
Dr3 = (d3(:,4:6)-[0,0,c3]);
D4 = [d4(:,1), -1 .* d4(:,2:3)];
Dr4 = (d4(:,4:6)-[0,0,c4]);
D5 = [d5(:,1), -1 .* d5(:,2:3)];
Dr5 = (d5(:,4:6)-[0,0,c5]);
D6 = [d6(:,1), -1 .* d6(:,2:3)];
Dr6 = (d6(:,4:6)-[0,0,c6]);
D7 = [d7(:,1), -1 .* d7(:,2:3)];
Dr7 = (d7(:,4:6)-[0,0,c7]);
D8 = [d8(:,1), -1 .* d8(:,2:3)];
Dr8 = -1.*d8(:,4:6);
Dr8(:,3) = d8(:,6)-d8(1,6)+180;

%timing
start_time = find(sqrt(sum((d1(:,1:3)-d2(:,1:3)).^2,2))<=2);
t_diff = d1(start_time(1),7)-d1(1,7);
ball(:,7) = (ball(:,7)-ball(1,7))./1000+t_diff;

% force
d1(:,7) = d1(:,7)-d1(1,7);

b0 = 0.0462486990765325;
b1 = 8992.55744491340;
c_ball = 1./(5./(ball(:,1:6).*0.0049).*10000-10000);
ball = [b1.*c_ball+ b0, ball(:,7)];

th = interp1(ball(:,7),ball(:,1),d1(:,7));
ind = interp1(ball(:,7),ball(:,2),d1(:,7));
ring = interp1(ball(:,7),ball(:,3),d1(:,7));
p1 = interp1(ball(:,7),ball(:,4),d1(:,7));
p2 = interp1(ball(:,7),ball(:,5),d1(:,7));
p3 = interp1(ball(:,7),ball(:,6),d1(:,7));

% c_th = 1./(5./(th.*0.0049).*10000-10000);
% th = b1.*c_th+ b0;
% c_ind = 1./(5./(ind.*0.0049).*10000-10000);
% ind = b1.*c_ind+ b0;
% c_ring = 1./(5./(ring.*0.0049).*10000-10000);
% ring = b1.*c_ring+ b0;
% c_p1 = 1./(5./(p1).*0.0049).*10000-10000);
% p1 = b1.*c_p1+ b0;
% c_p2 = 1./(5./(p2.*0.0049).*10000-10000);
% p2 = b1.*c_p2+ b0;
% c_p3 = 1./(5./(p3.*0.0049).*10000-10000);
% p3 = b1.*c_p3+ b0;

th(isnan(th))=0;
ind(isnan(ind))=0;
ring(isnan(ring))=0;
p1(isnan(p1))=0;
p2(isnan(p2))=0;
p3(isnan(p3))=0;

Cmax = max(max(ball(:,1:6)));
Cmin = min(min(ball(:,1:6)));
colors = hot(ceil(Cmax)-floor(Cmin)+1);
colormap hot

xmin = min(min([D1(:,1) D2(:,1) D3(:,1) D4(:,1) D5(:,1) D6(:,1) D7(:,1) D8(:,1)]));
xmax = max(max([D1(:,1) D2(:,1) D3(:,1) D4(:,1) D5(:,1) D6(:,1) D7(:,1) D8(:,1)]));
ymin = min(min([D1(:,2) D2(:,2) D3(:,2) D4(:,2) D5(:,2) D6(:,2) D7(:,2) D8(:,2)]));
ymax = max(max([D1(:,2) D2(:,2) D3(:,2) D4(:,2) D5(:,2) D6(:,2) D7(:,2) D8(:,2)]));
zmin = min(min([D1(:,3) D2(:,3) D3(:,3) D4(:,3) D5(:,3) D6(:,3) D7(:,3) D8(:,3)]));
zmax = max(max([D1(:,3) D2(:,3) D3(:,3) D4(:,3) D5(:,3) D6(:,3) D7(:,3) D8(:,3)]));

NF = [0,0,0];
for i = 1:10:length(D1)
    D1v =[0,0,1]*aer2rotm(Dr1(i,:))';
    D2v =[0,0,1]*aer2rotm(Dr2(i,:))';
    D3v =[0,0,1]*aer2rotm(Dr3(i,:))';
    D4v =[0,0,1]*aer2rotm(Dr4(i,:))';
    D5v =[0,0,1]*aer2rotm(Dr5(i,:))';
    D6v =[0,0,1]*aer2rotm(Dr6(i,:))';
    
    plot3(D1(i,1),D1(i,2),D1(i,3),'ro','MarkerSize',10,'MarkerFaceColor',colors(round(th(i))-floor(Cmin)+1,:));
    hold on
    quiver3(D1(i,1),D1(i,2),D1(i,3),D1v(1),D1v(2),D1v(3),round(th(i)),'r');
    plot3(D2(i,1),D2(i,2),D2(i,3),'go','MarkerSize',10,'MarkerFaceColor',colors(round(ind(i))-floor(Cmin)+1,:));
    quiver3(D2(i,1),D2(i,2),D2(i,3),D2v(1),D2v(2),D2v(3), round(ind(i)),'g');
    plot3(D3(i,1),D3(i,2),D3(i,3),'bo','MarkerSize',10,'MarkerFaceColor',colors(round(ring(i))-floor(Cmin)+1,:));
    quiver3(D3(i,1),D3(i,2),D3(i,3),D3v(1),D3v(2),D3v(3),round(ring(i)),'b');
    plot3(D4(i,1),D4(i,2),D4(i,3),'co','MarkerSize',10,'MarkerFaceColor',colors(round(p1(i))-floor(Cmin)+1,:));
    quiver3(D4(i,1),D4(i,2),D4(i,3),D4v(1),D4v(2),D4v(3),round(p1(i)),'c');
    plot3(D5(i,1),D5(i,2),D5(i,3),'mo','MarkerSize',10,'MarkerFaceColor',colors(round(p2(i))-floor(Cmin)+1,:));
    quiver3(D5(i,1),D5(i,2),D5(i,3),D5v(1),D5v(2),D5v(3),round(p2(i)),'m');
    plot3(D6(i,1),D6(i,2),D6(i,3),'yo','MarkerSize',10,'MarkerFaceColor',colors(round(p3(i))-floor(Cmin)+1,:));
    quiver3(D6(i,1),D6(i,2),D6(i,3),D6v(1),D6v(2),D6v(3),round(p3(i)),'y');
    plot3(D7(i,1),D7(i,2),D7(i,3),'o','MarkerSize',10,'MarkerEdgeColor',[1 0.5 0],'MarkerFaceColor',[0.5 0.5 0.5]);
    plot3([D1(i,1), D4(i,1), D7(i,1)],[D1(i,2), D4(i,2), D7(i,2)],[D1(i,3), D4(i,3), D7(i,3)],'k-');
    plot3([D2(i,1), D5(i,1), D7(i,1)],[D2(i,2), D5(i,2), D7(i,2)],[D2(i,3), D5(i,3), D7(i,3)],'k-');
    plot3([D3(i,1), D6(i,1), D7(i,1)],[D3(i,2), D6(i,2), D7(i,2)],[D3(i,3), D6(i,3), D7(i,3)],'k-');
    plot3([D5(i,1), D6(i,1)],[D5(i,2), D6(i,2)],[D5(i,3), D6(i,3)],'k-');
    plot3(D8(i,1),D8(i,2),D8(i,3),'ko');
    
    NF(1) = D1v(1)*th(i)+D2v(1)*ind(i)+D3v(1)*ring(i)+D4v(1)*p1(i)+D5v(1)*p2(i)+D6v(1)*p3(i);
    NF(2) = D1v(2)*th(i)+D2v(2)*ind(i)+D3v(2)*ring(i)+D4v(2)*p1(i)+D5v(2)*p2(i)+D6v(2)*p3(i);
    NF(3) = D1v(3)*th(i)+D2v(3)*ind(i)+D3v(3)*ring(i)+D4v(3)*p1(i)+D5v(3)*p2(i)+D6v(3)*p3(i);
    
    Fv.vertices = fv.vertices(:,:)*aer2rotm(Dr8(i,:))' + repmat(D8(i,:),r,1);
    patch(Fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
    camlight('headlight');
    material('dull');
    
    axis equal    
    view(45,30);
    xlim([xmin-5 xmax+5])
    ylim([ymin-5 ymax+5])
    zlim([zmin-5 zmax+5])
    ax = gca;
    ax.XTick = round(xmin-1):2:round(xmax+1);
    ax.YTick = round(ymin-1):2:round(ymax+1);
    ax.ZTick = round(zmin-2):2:round(zmax+1);
    grid on
    
    txt = ['time = ' num2str(d1(i,7)) 's'];
    text(xmin,ymin,zmin,txt);
    netf = sprintf('Net Force in:\nX: %d\nY: %d\nZ: %d',NF(1),NF(2),NF(3));
    text(xmax,ymin,zmin,netf)
    
    xlabel('x position')
    ylabel('y position')
    zlabel('z position')
    c=colorbar;
    caxis([Cmin Cmax]);
    c.Label.String = 'Force in Newtons';
    hold off
    M(i) = getframe(gcf);
    writeVideo(v,M(i));
end
close(v)