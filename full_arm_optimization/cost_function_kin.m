function F = cost_function_kin(a, input)

% OPTIMIZATIONROUTINE This function evaluates the overall objective function (F) for a
% specific set of DH parameters
%   inputs: Xi: column vector of DH parameters
%           m: number of discrete points along the trajectory
%           n_joints: number of revolute joints/2D or 3D space
%           w1 and w2: weights of subfunctions
%           pd: array of column vectors representing trajectory points
%           alpha: column vectory of alphas (other DH parameters)
%           constraints: q_min, q_max
%   outputs: f1, f2, and F. More can be added as needed.

global theta

m = input.m; % Number of samples
n_links = input.n_links;
% w1 = input.w1;
pd = input.pd;
% alpha = input.alpha;

% Constraints for theta
q_min = input.q_min; 
q_max = input.q_max; 

% Allocate array space
% for i = 1:size(Xi,1)
    theta   = zeros(n_links, m);
    pa      = zeros(4,4, m);       
    err_pos = zeros(1, m);
    f1      = 0;

    for j = 1:m
        % To maintain the continuity.
        if (j > 1)
            initial_q   = theta(:,j-1);
        else
            initial_q   = theta(:,j);
        end
        
        % For continuity, use the previous theta as initial theta.
        theta(:,j)     = IK(a, pd(:,:,j), initial_q, q_min, q_max);
        pa(:,:,j)      = FK([theta(:,j)' a]);
        
        % Position + Orientation 
        err_pos(:, j)  = sum((pd(1:3,end,j) - pa(1:3,end,j)).^2) + ...
                        sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,j))), rotm2quat(pa(1:3,1:3,j)))).^2);
        f1             = f1 + err_pos(:,j);

    end 
    F = f1; 
        
% end 
    
end
