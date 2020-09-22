
function plot_trajectory(poses, numsen, n_data) 
    figure(1)
    % close all;
    line = {'r-','k-'};
    q_ = {'r','b'};
    r = 7;
    
    for i=1:numsen
        p = reshape(poses(1:3,end,:,i),3,[]);
        
        plot3(p(1,1:n_data), p(2,1:n_data), p(3,1:n_data),line{i},'LineWidth',2)
        hold on
        for j=1:n_data
            if mod(j,r) == 0

                quiver3(poses(1,end,j,i), poses(2,end,j,i), poses(3,end,j,i), ...
                    poses(1,1:3,j,i)*[0 0 1]', poses(2,1:3,j,i)*[0 0 1]',...
                    poses(3,1:3,j,i)*[0 0 1]',q_{i},'LineWidth',1.5,'MaxHeadSize',0.5)

                hold on
            end
        end
    end
    
end