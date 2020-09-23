function [data_xyz] = tool_tip_pos(fileName,tool_length)
rot_mat = @(a,e,r)[cos(e)*cos(a) cos(e)*sin(a) -sin(e);-cos(r)*sin(a)+sin(r)*sin(e)*cos(a) cos(r)*cos(a)+sin(r)*sin(e)*sin(a) sin(r)*cos(e);sin(r)*sin(a)+cos(r)*sin(e)*cos(a) -sin(r)*cos(a)+cos(r)*sin(e)*sin(a) cos(r)*cos(e)];
data = fileName;%textread(fileName);
%data = data(1111:6722,:);% sample_path
%data = data(800:3778,:);% sample_path
%data = data(1505:44582,:); stiffness
%data = data(800:4500,:);% sample_path_suturing1
% tdata = data(800:end,:)%sutiring2)
for i=1:size(data,1)
   rot_angles = data(i,4:6)*pi/180;
   data_xyz(i,:) = data(i,1:3)' + rot_mat(rot_angles(1),rot_angles(2),rot_angles(3))'*tool_length';
end

figure
plot3(data_xyz(:,1),data_xyz(:,2),data_xyz(:,3))
axis equal

end