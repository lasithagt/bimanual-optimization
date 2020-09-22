%% Vector function for IK
function [q_ret] = IK_SE3_MR(Slist, M, new_cart_pose) 
    
    global input
    poses = new_cart_pose;
    p     = reshape(poses,4,4,[]);

    guess = zeros(7,1) + 0.1*rand(7,1);
    
    n = size(p,3);
    
    q_ret = zeros(7,n);
    eomg = 0.01; ev = 0.001;
    
    for i=1:n
        T          = p(:,:,i);
        theta_hat  = IKinSpace_modified(Slist, M, T, guess, eomg, ev, input);
        
        % impose hard limits
%         m_min = input.q_min' > theta_hat;
%         theta_hat(m_min) = input.q_min(m_min);
%         
%         m_max = input.q_max' < theta_hat;
%         theta_hat(m_max) = input.q_max(m_max);
        
        q_ret(:,i) = theta_hat;
        guess      = theta_hat;
    end
    
end






