% This function takes in current q position, new_cartesian point and vel_d
function [q_ret,err] = IK_invJac(x, curr_q, new_cart_pose, vel_d) 

    global input
    iter_max = 200;
    pa = FK(x, curr_q);
    pd = new_cart_pose;
    
    v_p = vel_d(1:3);
    v_o = vel_d(4:end);
    
    % make it changable 
    Ko = 0.7; Kp = -3.0;
    q_ret = zeros(input.n_arms,input.n_links);
    
    % for the dual arm, coordinate frames are rotated.
    T_L = input.T_L;
    T_R = input.T_R;
    
    % first - Left arm
    T = zeros(4,4,2); T(:,:,1) = T_L; T(:,:,2) = T_R;
    
    for i = 1:input.n_arms
        curr_p  = pa;
        err     = 100; 
        err_rel = 100; 
        q_new   = curr_q(i,:)';
        iter = 0;
        
        while(err_rel > 1e-5 && err > 1e-8 && iter<=iter_max)
            iter = iter + 1;
            e_p = (curr_p(1:3,end,i) - pd(1:3,end,i));
            e_o = [1 0 0 0]' - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,i))), rotm2quat(curr_p(1:3,1:3,i)))';
            
            % check for singularities and adds dampening if so.
            R  = T(1:3,1:3,i);
            RR = [R zeros(3,3);zeros(3,3) R];
            J  = RR * Jacob(curr_q(i,:), x(5:end));
            
            % dealing with singularities
            if (rcond(J*J') < 0.01) 
                J_m = J'/(J*J' + 0.01 * eye(6)) ;
            else
                J_m = J'/(J*J');
            end
            
            % psudo inverse. 
            qdot       = J_m * [(v_p + Kp * e_p);(v_o + Ko * e_o(2:end))] + redundancy_optimizer(input, J, q_new);
            q_new      = q_new + qdot * 0.001;
            q_ret(i,:) = q_new;
            curr_p     = FK(x, q_ret);
            err_n      = sqrt(sum((pd(1:3,end,i) - curr_p(1:3,end,i)).^2) + ...
                            1.4*sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,i))), rotm2quat(curr_p(1:3,1:3,i))))).^2);
            
            % exit if error begins to be incremental
            if (err_n > err)
                break;
            end
            
            err_rel = abs(err_n - err)./err;
            err = err_n;
            
        end
        
    
    end
     
end

%% TODO: optimizes the null space to increase the manipulability and stay
% away from joint limits
function q0 = redundancy_optimizer(input, J, q)
    % to stay away from joint limits.
    k = 0.1;
    q_mid       = (input.q_min + input.q_max)./2;
    
    % w           = -(1/2*input.n_links) * sum((q - q_mid)./(input.q_max - input.q_min)).^2;
    
    q_lim_grad  = -(1/input.n_links) * ((q' - q_mid)./(input.q_max - input.q_min));
    q0          = (eye(7)-J'/(J*J')*J) * k * q_lim_grad';
    
    % TODO:
    % to maximize the manipulability measure
    
    
    % to avoid self collisions
end

%% 
function dis_collision()

end




