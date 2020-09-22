function plot_D(T_data, JJ_data)
    x = reshape(T_data(1,4,1:1:end),1,[]);
    y = reshape(T_data(2,4,1:1:end),1,[]);
    z = reshape(T_data(3,4,1:1:end),1,[]);
    
    det_store_pos = zeros(1,size(JJ_data,3));
    det_store_orient = zeros(1,size(JJ_data,3));
    
    for i = 1:size(JJ_data,3)
%         det_store_pos(i) = det(JJ_data(1:3,:,i) * JJ_data(1:3,:,i)');
        e = eig(JJ_data(1:3,:,i) * JJ_data(1:3,:,i)');
        det_store_pos(i) = min(e)/max(e);
%         det_store_orient(i) = det(JJ_data(4:6,:,i) * JJ_data(4:6,:,i)');
        e_o = eig(JJ_data(4:6,:,i) * JJ_data(4:6,:,i)');
        det_store_orient(i) = min(e_o)/max(e_o);
    end
    
%     [X, Y, Z] = meshgrid(x,y,z);
    
    close all
    
    figure(1)
%     scatter3(x(mask_pos),y(mask_pos),z(mask_pos),50,det_store_pos(mask_pos),'filled')    % draw the scatter plot
    mask = x<0 | x>0.1;
    x(mask) = []; y(mask) = []; z(mask) = []; det_store_pos(mask) = []; det_store_orient(mask) = [];
    
    det_store_pos = det_store_pos./ max(det_store_pos);
    mask_pos = find(det_store_pos>0.3 & det_store_pos<1);
    mas_orient = find(det_store_orient>0.03 & det_store_orient<0.1);
    
    scatter3(x(mask_pos), y(mask_pos), z(mask_pos),'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75])    % draw the scatter plot

    ax = gca;
    ax.XDir = 'reverse';
    view(-31,14)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
%     cb = colorbar;   
    hold on
%     figure(2)
%     scatter3(x(mas_orient),y(mas_orient),z(mas_orient),50,det_store_orient(mas_orient),'filled')    % draw the scatter plot
    scatter3(x(mas_orient),y(mas_orient),z(mas_orient),'MarkerEdgeColor','b',...
        'MarkerFaceColor',[0.75 0 .75])    % draw the scatter plot

    ax = gca;
    ax.XDir = 'reverse';
    view(-31,14)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
    hold on
    
    figure(3)
%     d = -1:0.05:1;
    [zq,yq,xq] = meshgrid(linspace(-0.4,1.2,500), linspace(-1,1,500), 0.06);
    vq = griddata(z(mask_pos), y(mask_pos), x(mask_pos), det_store_pos(mask_pos),zq,yq,xq);
    mesh(yq,zq,xq,vq,'FaceAlpha','0.5', 'FaceColor', 'interp', 'EdgeColor' ,'none')

    figure(4)
%     d = -1:0.05:1;
    [zq,yq,xq] = meshgrid(linspace(-0.4,1.2,500), linspace(-1,1,500), 0.06);
    vq = griddata(z(mas_orient), y(mas_orient), x(mas_orient), det_store_orient(mas_orient),zq,yq,xq);
    mesh(yq,zq,xq,vq,'FaceAlpha','0.5', 'FaceColor', 'flat', 'EdgeColor' ,'none')


%     cb = colorbar;   
    
%     figure(3)
%     xslice = [5 9.9];                               % define the cross sections to view
%     yslice = 3;
%     zslice = ([-3 0]);
% 
%     slice(x, y, z, det_store_orient(1:1:end), xslice, yslice, zslice)    % display the slices
%     ylim([-3 3])
%     view(-34,24)
% 
%     cb = colorbar;                                  % create and label the colorbar
%     cb.Label.String = 'Temperature, C';


%     plot3(x,y,z,'r.')
end