
function plot_data(em_data, is_animate)
    %% plotting
    % TODO: Extract force data at each point. (for visualization)
    % close all;
    
    line = {'r-','k-'};
    q_   = {'r','b'};
    r    = 10;

    numsen = size(em_data,3);
    n_data = size(em_data,2);
    h1 = animatedline('Marker','.','Color','r');
    h2 = animatedline('Marker','.','Color','k');


    xlabel('x position')
    ylabel('y position')
    zlabel('z position')

    grid on
    h = [h1, h2];
    
    for i = 1:numsen
        if (is_animate)
            for j=1:n_data
                if mod(j,r) == 0
                    plot3(em_data(1,j,i), em_data(2,j,i), em_data(3,j,i),'.')
                    quiver3(em_data(1,j,i),em_data(2,j,i),em_data(3,j,i), ...
                        -em_data(4,j,i),-em_data(5,j,i), -em_data(6,j,i),q_{i}, 'LineWidth',0.5,'MaxHeadSize',0.05)
                    hold on
                    % axis([0 30 -10 10 -10 25])
                    pause(0.01)
                end
            end
        else
            plot3(em_data(1,:,i), em_data(2,:,i), em_data(3,:,i),'.')
            quiver3(em_data(1,:,i),em_data(2,:,i),em_data(3,:,i), ...
                em_data(4,:,i),em_data(5,:,i), em_data(6,:,i),q_{i}, 'LineWidth',0.5,'MaxHeadSize',0.05)
            hold on
            % axis([0 30 -10 10 -10 25])
        end
    end
    
end