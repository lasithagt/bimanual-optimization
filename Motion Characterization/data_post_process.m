function [d, p_m, p_std, o_m, o_std] = data_post_process(data_file)

    % load data
    d = load(data_file);
    new_pp_data = d.em_data_adj;

    % take the mean of the position 
    % calculatet eh mean of x,y and z positions
    d_p   = d.em_data_adj; d_p = d_p(1:3,:);
    p_m   = mean(d_p,2);
    p_std = std(d_p,0,2);
    new_pp_data(1:3,:) = new_pp_data(1:3,:) - p_m;

    % calculatet eh mean of x,y and z orientations
    d_o   = d.em_data_adj; d_o = d_o(4:6,:);
    o_m   = mean(d_o,2);
    o_std = std(d_o,0,2);
    new_pp_data(4:6,:) = new_pp_data(4:6,:) - o_m;
    
    % options = statset('Display','final');
    % gm = fitgmdist(X,2,'Options',options)
    
    % resample in the abstracted data parameters
    d.em_data_adj = new_pp_data;
    stats       = [p_m, p_std, o_m, o_std];
    
    % save as a new data structure
    name = split(data_file, '.mat');
    name = strcat(name{1}, '_new_pp.mat');
    em_data_adj = d.em_data_adj;
    em_tracker_vel = d.em_tracker_vel;
    tool_tip_cal = d.tool_tip_cal;
    save(name, 'em_data_adj', 'em_tracker_vel', 'tool_tip_cal','stats')
    
    
    % 
    
end