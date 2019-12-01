% This function takes in current q position, new_cartesian point and vel_d
function [q_ret,err] = IK_invJac(x, curr_q, new_cart_pose, vel_d) 

    global input
    iter_max = 500;
    pa = FK(x, curr_q);
    pd = new_cart_pose;
    
    v_p = vel_d(1:3);
    v_o = vel_d(4:end);
    
    % make it changable 
    Ko = 0.7; Kp = -2.7;
    q_ret = zeros(input.n_arms,input.n_links);
    
    % for the dual arm, coordinate frames are rotated.
    T_L = input.T_L;
    T_R = input.T_R;
    
    % first - Left arm
    T = zeros(4,4,2); T(:,:,1) = T_L; T(:,:,2) = T_R;
    
    for i = 1:input.n_arms
        curr_p  = pa;
        err     = 100; %;norm(pa(:,:,i) - pd(:,:,i)).^2;
        err_rel = 100; %norm(pa(:,:,i) - pd(:,:,i)).^2;
        q_new   = curr_q(i,:)';
        iter = 0;
        while(err_rel > 1e-5 && err > 1e-8 && iter<=iter_max)
            iter = iter + 1;
            e_p = (curr_p(1:3,end,i) - pd(1:3,end,i));
            e_o = [1 0 0 0]' - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,i))), rotm2quat(curr_p(1:3,1:3,i)))';
%             e_o = [0 0 0 0]';
            % TODO: solve for q_dot that gives more manipulability.
            
            % check for singularities and adds dampening if so.
            R  = T(1:3,1:3,i);
            RR = [R zeros(3,3);zeros(3,3) R];
            J  = RR * Jacob(curr_q(i,:), x(5:end));
            
            % dealing with singularities
            if (rcond(J*J') < 0.001) 
                J_m = J'/(J*J' + 0.001 * eye(6)) ;
            else
                J_m = J'/(J*J');
            end
            
            % use psudo inverse for now. 
            qdot    = J_m * [(v_p + Kp * e_p);(v_o + Ko * e_o(2:end))];
            q_new   = q_new + qdot * 0.001;
            q_ret(i,:) = q_new;
            curr_p  = FK(x, q_ret);
            err_n = sqrt(sum((pd(1:3,end,i) - curr_p(1:3,end,i)).^2) + ...
                            sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,i))), rotm2quat(curr_p(1:3,1:3,i))))).^2);
            err_rel = abs(err_n - err)./err;
            err = err_n;
            
        end
        
    
    end
     
end

%% TODO: optimizes the null space to increase the manipulability and stay
% away from joint limits
function q = redundancy_optimizer(in, q)
% to stay away from joint limits.

        q_lim_grad = exp((q - input.q_min)) + exp((q - input.q_max))
        q0 = null(J) * q_lim_grad
end