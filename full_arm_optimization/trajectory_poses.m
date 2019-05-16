function [p, qdd_des] = trajectory_poses(m)
    % Fix the position
    p = zeros(4,4,m);
    p_des = [2,1,0]';
    
    % Get orientations 
    init_or  = 1.5*pi/4;
    final_or = -pi/4;
    p_o      = orientation_sampler(init_or, final_or, m);
    
    % Construct p
    for i = 1:m
        p(:,:,i) = [quat2rotm(p_o(:,i)') p_des;0 0 0 1];
    end
    qdd_des = [1 1 1 1;1 0.1 2 1];
end

% This only works for planer rotations
function p = orientation_sampler(init, final, m)
    ini_p = rotm2quat(rotz(init));
    final_p = rotm2quat(rotz(final));
    inter_p = zeros(4,m);
    
    for i = 1:m
        temp = ini_p' + (final_p-ini_p)'*(i/m); 
        inter_p(:,i) = temp./norm(temp);
    end
    p = inter_p;
end