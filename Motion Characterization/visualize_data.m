
close all;
clear;
% Load the data from a mat file
cali_data = readNPY('Data/calibration_data.npy');
ws_d      = rosbag('Data/2019-05-25-17-38-51.bag');

em_data = select(ws_d,'Topic','/EMdata');
msg_em = cell2mat(readMessages(em_data,'DataFormat','struct'));
em_ = vertcat(msg_em(1:end).Poses);
numsen = size(em_,2);
n_data = size(em_,1);

em_data_mat = zeros(7,size(em_,1),numsen);

for i = 1:numsen
    e_temp = vertcat(em_(:,i).Pose);
    e_temp_pos = vertcat(e_temp.Position);
    e_temp_ori = vertcat(e_temp.Orientation);
    em_data_mat(1,:,i) = horzcat(e_temp_pos.X);
    em_data_mat(2,:,i) = horzcat(e_temp_pos.Y);
    em_data_mat(3,:,i) = horzcat(e_temp_pos.Z);
    % since using eul
    em_data_mat(4,:,i) = horzcat(e_temp_ori.Z);
    em_data_mat(5,:,i) = horzcat(e_temp_ori.Y);
    em_data_mat(6,:,i) = horzcat(e_temp_ori.X);
    em_data_mat(7,:,i) = horzcat(e_temp_ori.W);
end


% Extract cal data

% Adjusted data
em_data_adj = zeros(6,n_data,numsen);
tool_tip_cal = {};
for i = 1:numsen

    sensor_i_data = cali_data(:,:,i);
    pos_xyz = sensor_i_data(1:3,:)';
%     temp = sensor_i_data(4,:);
%     sensor_i_data(4:end-1,:) = sensor_i_data(5:end,:);
%     sensor_i_data(end,:) = temp;
%     ang_eul = quat2eul(sensor_i_data(4:end,:)','ZYX') ;
    
    ang_eul = sensor_i_data(4:end-1,:)';
    % compute the tool tip transformation matrix for all the instruments
    [tool_tip_cal{i}, fval]  = tool_tip_approx_([pos_xyz ang_eul]);
    
    fval
    % Extract position and angular data and get the tool tip position
    em_data_mat_pos = em_data_mat(1:3,:,i)';
%     em_data_mat_ang = quat2eul(em_data_mat(4:end,:,i)');
    em_data_mat_ang = em_data_mat(4:end-1,:,i)';
    
    for k = 1:n_data
        em_data_adj(1:3,k,i) = em_data_mat_pos(k,:)' + eul2rotm(em_data_mat_ang(k,:),'ZYX')*tool_tip_cal{i}(1:3)';
        temp = eul2rotm(em_data_mat_ang(k,:),'ZYX') * [0 0 -1]';
        em_data_adj(4:end,k,i) = temp./norm(temp);
    end

end

save('EM_data.mat','em_data_adj', 'tool_tip_cal')

%% plotting
figure(1)
% TODO: Extract force data at each point. (for visualization)
% close all;
line = {'r-','k-'};
q_ = {'r','b'};
r = 50;
for i=1:numsen
    plot3(em_data_adj(1,:,i), em_data_adj(2,:,i), em_data_adj(3,:,i),line{i})
    hold on
    for j=1:n_data
        if mod(j,r) == 0

            quiver3(em_data_adj(1,j,i),em_data_adj(2,j,i),em_data_adj(3,j,i), ...
                em_data_adj(4,j,i),em_data_adj(5,j,i),em_data_adj(6,j,i),q_{i},'LineWidth',2,'MaxHeadSize',0.5)

            hold on
        end
    end
end

% view(45,30);
xlabel('x position')
ylabel('y position')
zlabel('z position')
% axis([50 50 -50 50 -50 50])
grid on
hold off
