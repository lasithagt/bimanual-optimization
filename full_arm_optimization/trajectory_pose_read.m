
% Takes data from em_tracker and convert it to a new user defined base.
function poses = trajectory_pose_read(data_file, m)

    n_poses = m;
    em_data = load(data_file);
    numsen  = size(em_data.em_data_adj,3);
    n_data  = size(em_data.em_data_adj,2);
    em_data_adj = em_data.em_data_adj;

    % convert to z pointing up and changing the position of the base
    R_b =  rotz(pi/2)*rotx(pi)*rotz(pi) ;
    p_b = [0, 10.15,0]';

    em_data_new_b = zeros(size(em_data_adj));
    
    for i=1:numsen
        new_p = R_b * em_data_adj(1:3,:,i) + p_b;
        em_data_new_b(1:3,:,i) = new_p;
        
        for j = 1:n_data
            new_R = rotm2eul(R_b * eul2rotm(em_data_adj(4:end,j,i)','ZYX'),'ZYX')';
            em_data_new_b(4:end,j,i) = new_R;
        end
    end

    poses = zeros(4,4,n_poses,numsen);
    poses(4,4,:,:) = 1;
    % give requested number of data points from the entire trajectory.

    for i = 1:numsen
        t = floor(linspace(1, n_data,n_poses));
        poses(1:3,end,:,i) = em_data_new_b(1:3,t,i);
        for k = 1:n_poses
            poses(1:3,1:3,k,i) = rotz(pi) * eul2rotm(em_data_new_b(4:end,t(k),i)', 'ZYX');
        end
    end
        
    % For testing purposes
%     plot_data(poses, numsen, n_poses)
    
end

function plot_data(poses, numsen, n_data) 
    figure(1)
    % close all;
    line = {'r-','k-'};
    q_ = {'r','b'};
    r = 7;
    
    for i=1:numsen
        p = reshape(poses(1:3,end,:,i),3,[]);
        
        plot3(p(1,:), p(2,:), p(3,:),line{i})
        hold on
        for j=1:n_data
            if mod(j,r) == 0

                quiver3(poses(1,end,j,i), poses(2,end,j,i), poses(3,end,j,i), ...
                    poses(1,1:3,j,i)*[0 0 1]', poses(2,1:3,j,i)*[0 0 1]',...
                    poses(3,1:3,j,i)*[0 0 1]',q_{i},'LineWidth',2,'MaxHeadSize',0.5)

                hold on
            end
        end
    end
    
end