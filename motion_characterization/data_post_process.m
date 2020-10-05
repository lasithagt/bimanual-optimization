function [d, p_m, p_std, o_m, o_std] = data_post_process(data_file)

    data_ = load(data_file);
    data  = data_.em_data_adj;
      
    %% take the mean of the position 
    % calculatet eh mean of x,y and z positions
    d_p         = data(1:3,:);
    p_m         = mean(d_p,2);
    p_std       = std(d_p,0,2);
    data(1:3,:) = data(1:3,:) - p_m;

    %% calculatet eh mean of x,y and z orientations
    % convert orientations to quternions.
    d_o         = data(4:6,:);
    o_m         = mean(d_o,2);
    o_std       = std(d_o,0,2);
    data(4:6,:) = data(4:6,:) - 0*o_m;
    
    %% plot data
    is_animate = false;
    figure(1)
    plot_data(data, is_animate);
    xlabel('X axis');
    ylabel('Y axis');
    zlabel('Z axis');
    
    %% abstract data
    rng(1); % For reproducibility

    options = statset('Display','final','MaxIter',1000);
    n = 3;
    for n_sensor = 1:2
        % sensor 1
        data_sensor = data(1:3, :, n_sensor);
        gm = fitgmdist(data_sensor', n,'Options',options);

    %     AIC = zeros(1,4);
    %     GMModels = cell(1,4);
    %     options = statset('MaxIter',1000);
    %     for k = 1:10
    %         GMModels{k} = fitgmdist(data_1', k, 'Options',options,'CovarianceType','diagonal');
    %         AIC(k)= GMModels{k}.AIC;
    %     end
    % 
    %     [minAIC,numComponents] = min(AIC);
    %     numComponents

        %% plot 3D ellipsoids
        for i = 1:n
            mu = gm.mu(i, :);
            C  = gm.Sigma(:,:,i);

            h = plot_gaussian_ellipsoid(mu, C);
            set(h,'facealpha',0.9);
    %     view(129,36); set(gca,'proj','perspective'); grid on; 
    %     grid on; axis equal; axis tight;
        end
    
    end

    
    %% save data
    % resample in the abstracted data parameters
    d.em_data_adj = data;
    stats         = [p_m, p_std, o_m, o_std];
    
    % save as a new data structure
    name = split(data_file, '.mat');
    name = strcat(name{1}, '_new_pp.mat');
    em_data_adj = d.em_data_adj;
    em_tracker_vel = data_.em_tracker_vel;
    tool_tip_cal = data_.tool_tip_cal;
    save(name, 'em_data_adj', 'em_tracker_vel', 'tool_tip_cal','stats')
    
    
end