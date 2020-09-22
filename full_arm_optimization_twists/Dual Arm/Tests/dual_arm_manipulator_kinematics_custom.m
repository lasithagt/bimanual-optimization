 
function [pose_K, pose_L, pose_R, Jst_L, Jst_R, J_K_s] = dual_arm_manipulator_kinematics_custom(p, theta_mini_L, theta_mini_R, theta_KUKA)

    
    M_k = p{1}; S_k = p{2}; M_r = p{3}; S_r = p{4}; M_l = p{5}; S_l = p{6};
    
    %% manipulator twists (kuka)
    pose_K   = FKinSpace(M_k, S_k, theta_KUKA);
    J_K_s    = JacobianSpace(S_k,theta_KUKA);
    % J_K_b    = Adjoint(inv(pose_K)) * J_K_s;
    
    %% mini-arm kinematics, right
    pose_R = FKinSpace(M_r, S_r, theta_mini_R);
    Jst_R  = JacobianSpace(S_r,theta_mini_R);
    
    %% mini-arm kinematics, left
    pose_L = FKinSpace(M_l, S_l, theta_mini_L);
    Jst_L  = JacobianSpace(S_l,theta_mini_L);
    
end
