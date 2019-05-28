
% Takes data from em_tracker and convert it to a new user defined base.
function poses = trajectory_pose_read(data_file, m)

    n_poses = m;
    em_data = load(data_file);
    numsen  = size(em_data.em_data_adj,3);
    n_data  = size(em_data.em_data_adj,2);
    em_data_adj = em_data.em_data_adj;

    % convert to z pointing up and changing the position of the base
    R_b =  rotx(rad2deg(pi))*rotz(rad2deg(pi)) ;
    p_b = [10.15, 0,0]';

    em_data_new_b = zeros(size(em_data_adj));
    
    for i=1:numsen
        new_p = R_b * em_data_adj(1:3,:,i) + p_b;
        em_data_new_b(1:3,:,i) = new_p;
        
        new_R = R_b * rotm2eul(eul2rotm(em_data_adj(4:end,:,i)','ZYX'),'ZYX')';
        em_data_new_b(4:end,:,i) = new_R;
    end

    poses = ones(4,4,n_poses,numsen);
    % give requested number of data points from the entire trajectory.

    for j = 1:numsen
        t = floor(linspace(1, n_data,n_poses));
        poses(1:3,end,:,j) = em_data_new_b(1:3,t,j);
        poses(1:3,1:3,:,j) = eul2rotm(em_data_new_b(4:end,t,j)', 'ZYX');
    end
        
    
    % For testing purposes
    % plot_data(em_data_new_b, numsen, n_data)
    
end

function plot_data(em_data_adj, numsen, n_data) 
    figure(1)
    % close all;
    line = {'r-','k-'};
    q_ = {'r','b'};
    r = 10;
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
    
end