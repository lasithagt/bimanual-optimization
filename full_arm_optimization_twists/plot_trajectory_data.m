function plot_trajectory_data(poses, numsen, n_data) 
    line = {'r-','b-'};
    q_   = {'r','b'};
    r    = 7;
    
    for i=1:numsen
        p = reshape(poses(1:3,end,:,i),3,[]);
        
        plot3(p(1,:), p(2,:), p(3,:),line{i},'LineWidth',2)
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
    xlabel('X axis')
    ylabel('Y axis')
    zlabel('Z axis')
    
end