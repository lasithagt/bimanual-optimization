
close all;
clear;

% addpath('../npy-matlab/npy-matlab')

% Load the data from a mat file
calibration = 1;
if (calibration)
    cali_data = readNPY('Data/EM_tracker/calibration_data_soldering.npy');
    % ws_d      = rosbag('Data/2019-05-30-15-51-10.bag');
    ws_d      = rosbag('Data/EM_tracker/soldering.bag');


    em_data = select(ws_d,'Topic','/EMdata');
    msg_em  = cell2mat(readMessages(em_data,'DataFormat','struct'));
    em_     = vertcat(msg_em(1:end).Poses);
    
    % numsen  = size(em_,2);
    numsen  = 2;
    n_data  = size(em_,1);

    em_data_mat = zeros(7,size(em_,1),numsen);

    for i = 1:numsen
        e_temp = vertcat(em_(:,i+1).Pose);
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
    em_tracker_vel = zeros(3, n_data, numsen);

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
        [tool_tip_cal{i}, fval]  = tool_tip_approx_([pos_xyz ang_eul],i);


        fprintf("Calibration Tool %d error: %d\n",[i, fval])
        % Extract position and angular data and get the tool tip position
        em_data_mat_pos = em_data_mat(1:3,:,i)';
        % em_data_mat_ang = quat2eul(em_data_mat(4:end,:,i)');
        em_data_mat_ang = em_data_mat(4:end-1,:,i)';

        for k = 1:n_data
            em_data_adj(1:3,k,i)   = em_data_mat_pos(k,:)' + eul2rotm(em_data_mat_ang(k,:),'ZYX')*tool_tip_cal{i}(1:3)';
            temp                   = eul2rotm(em_data_mat_ang(k,:),'ZYX') * [1 0 0]';
            em_data_adj(4:end,k,i) = temp;
        end
        em_tracker_vel(1,:,i) = getDerivative(em_data_adj(1,:,i)',100.0);
        em_tracker_vel(2,:,i) = getDerivative(em_data_adj(2,:,i)',100.0);
        em_tracker_vel(3,:,i) = getDerivative(em_data_adj(3,:,i)',100.0);

    end

    save('EM_data.mat','em_data_adj', 'tool_tip_cal', 'em_tracker_vel')
else 
    load('EM_data.mat')
end


%% plotting
figure(3)
% TODO: Extract force data at each point. (for visualization)
% close all;
line = {'r-','k-'};
q_ = {'r','b'};
r = 50;

numsen = size(em_data_adj,3);
n_data = size(em_data_adj,2);
h1 = animatedline('Marker','.','Color','r');
h2 = animatedline('Marker','.','Color','k');


xlabel('x position')
ylabel('y position')
zlabel('z position')

grid on
h = [h1,h2];
for j=1:n_data
    j;
    for i=1:numsen
        if mod(j,r) == 0
            
%             addpoints(h(i), em_data_adj(1,j,i), em_data_adj(2,j,i), em_data_adj(3,j,i));
            plot3(em_data_adj(1,j,i), em_data_adj(2,j,i), em_data_adj(3,j,i),'.')
            quiver3(em_data_adj(1,j,i),em_data_adj(2,j,i),em_data_adj(3,j,i), ...
                -em_data_adj(4,j,i),-em_data_adj(5,j,i),-em_data_adj(6,j,i),q_{i}, 'LineWidth',0.5,'MaxHeadSize',0.05)
% 
            hold on
            %drawnow
            axis([0 30 -10 10 -10 25])
            pause(0.01)
        end
    end
end

figure(4)
% TODO: Extract force data at each point. (for visualization)
% close all;

    subplot(3,2,1)
    plot(em_tracker_vel(1,:,1))
    
    subplot(3,2,2)
    plot(em_tracker_vel(1,:,2))
    
    subplot(3,2,3)
    plot(em_tracker_vel(2,:,1))
    
    subplot(3,2,4)
    plot(em_tracker_vel(2,:,2))
    
    subplot(3,2,5)
    plot(em_tracker_vel(3,:,1))
    
    subplot(3,2,6)
    plot(em_tracker_vel(3,:,2))

%% This funtion filters out the derivative of the velocity to give the. This should be called in the constructor
% accleration using a derivative filter
function [v_d] = getDerivative(qd, Fs)

    if nargin < 2
        Fs = 100;
    end
    
    d = designfilt('lowpassfir', ...
        'PassbandFrequency',0.1,'StopbandFrequency',0.9, ...
        'PassbandRipple',1,'StopbandAttenuation',60, ...
        'DesignMethod','equiripple','SampleRate',Fs);

    v_d = zeros(size(qd,1), size(qd,2));
    for i = 1:size(qd,2)
        v_d(2:1:end,i) = filtfilt(d, diff(qd(:,i)))/(1/Fs);
        v_d(1,i) = (v_d(2,i) + v_d(1,i))/2;
    end
    
end