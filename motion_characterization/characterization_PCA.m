function characterization_PCA
    close all
    addpath('./data-processed/')
    data = load('path_tracking.mat');
    
    right_arm = data.em_data_adj(:,:,2);
    right_arm_vel = data.em_tracker_vel(:,:,2);
    
    right_arm = [right_arm;right_arm_vel];
    
    [wcoeff,~,latent,~,explained] = pca(right_arm', 'VariableWeights','variance')
    
    size(right_arm)
    right_arm_1 = right_arm .* wcoeff(:,1);
    right_arm_2 = right_arm .* wcoeff(:,2);
    
    figure(1)
    plot3(right_arm(1,:), right_arm(2,:), right_arm(3,:), 'k', 'LineWidth', 2)
    hold on
    plot3(right_arm_1(1,:), right_arm_1(2,:), right_arm_1(3,:), 'k')
    hold on
    plot3(right_arm_2(1,:), right_arm_2(2,:), right_arm_2(3,:), 'b')

end

function generate_features(data)
    for 
        
    end
end