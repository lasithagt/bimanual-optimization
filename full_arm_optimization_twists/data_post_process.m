function data_post_process(data_file)

% load data
d = load(data_file);

% take the mean of the position 
% calculatet eh mean of x,y and z positions
d_p = d.em_data_adj; d_p = d_p(1:3,:);
p_m = mean(d_p,2);

% calculatet eh mean of x,y and z orientations
d_o = d.em_data_adj; d_o = d_o(1:3,:);
p_m = mean(d_o,2);

end