%% Vector function for IK
function [q_ret] = IK_SE3(Slist, M, new_cart_pose) 

    poses = new_cart_pose;
    p     = reshape(poses,4,4,[]);
    
    % rng default
    guess = zeros(7,1) + 0.1*rand(7,1);
    
    n = size(p,3);
    
    q_ret = zeros(7,n);
    eomg = 0.01; ev = 0.001;
    
    for i=1:n
        T          = p(:,:,i);
        theta_hat  = IKinSpace_modified(Slist, M, T, guess, eomg, ev);
        q_ret(:,i) = theta_hat;
        guess      = theta_hat;
    end
    
end






