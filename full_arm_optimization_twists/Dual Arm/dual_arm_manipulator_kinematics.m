 
function [pose_K, pose_L, pose_R, Jst_R, Jst_L, J_K_s, J_K_b, M_K, Slist_K] = dual_arm_manipulator_kinematics(theta_mini_R, theta_mini_L, theta_KUKA)

    %% manipulator twists (kuka)
    W1 = [0 0 1]'; W2 = [0 -1 0]';
    w1 = W1; q1 = [0;0;0.2025];
    w2 = W2; q2 = [0;0;0.2025];
    w3 = W1; q3 = [0;0;0.2025];
    w4 = -W2; q4 = [0;0;0.2025+0.42];
    w5 = W1; q5 = [0;0;0.2025+0.42];
    w6 = W2; q6 = [0;0;0.2025+0.42+0.4];
    w7 = W1; q7 = [0;0;0.2025+0.42+0.4+0.126];

    R = rotx(pi)*roty(deg2rad(48));
    g_st = [[R;0 0 0],[0;-0.0893;0.2025+0.42+0.4+0.126+0.2086;1]];
%     g_st = [1 0 0 0;0 1 0 -0.0893;0 0 1 0.2025+0.42+0.4+0.126+0.2086;0 0 0 1];
    M_K = g_st;

    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);

    S        = [S1, S2, S3, S4, S5, S6, S7];
    Slist_K = S;
    pose_K   = FKinSpace(g_st, S, theta_KUKA);
    J_K_s    = JacobianSpace(S,theta_KUKA);
    J_K_b    = Adjoint(inv(pose_K)) * J_K_s;
    %% mini-arm kinematics, right

    w1 = [0 -1 0]'; q1 = [0;-0.0807;0.0];
    w2 = [0 0 -1]'; q2 = [0;-0.0807-0.0893;0];
    w3 = [1 0 0]';  q3 = [0.1706;-0.0807-0.0893;0];
    w4 = [0 0 -1]'; q4 = [0.1706+0.1055;-0.0807-0.0893;-0.0291];
    w5 = [0 -1 0]';  q5 = [0.1706+0.1055+0.183;-0.0807-0.0893;-0.0291];
    w6 = [0 0 1]';   q6 = [0.1706+0.1055+0.183;-0.0807-0.0893;-0.0291];
    w7 = [1 0 0]';   q7 = [0.1706+0.1055+0.183;-0.0807-0.0893;-0.0291];

    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);

    M = [[1, 0, 0, 0.1706+0.1055+0.183]; [0, 1, 0, -0.0807-0.0893]; [0, 0, 1, -0.0291]; [0, 0, 0, 1]];

    S = [S1 S2 S3 S4 S5 S6 S7];
    pose_R = FKinSpace(M, S, theta_mini_R);
    Jst_R = JacobianSpace(S,theta_mini_R);
    
    %% mini-arm kinematics, left

    w1 = [0 1 0]'; q1 = [0;0.0807;0.0];
    w2 = [0 0 -1]'; q2 = [0;0.0807+0.0893;0];
    w3 = [1 0 0]';  q3 = [0.1706;+0.0807+0.0893;0];
    w4 = [0 0 -1]'; q4 = [0.1706+0.1055;+0.0807+0.0893;-0.0291];
    w5 = [0 1 0]';  q5 = [0.1706+0.1055+0.183;+0.0807+0.0893;-0.0291];
    w6 = [0 0 1]';   q6 = [0.1706+0.1055+0.183;+0.0807+0.0893;-0.0291];
    w7 = [1 0 0]';   q7 = [0.1706+0.1055+0.183;+0.0807+0.0893;-0.0291];

    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);

    M = [[1, 0, 0, 0.1706+0.1055+0.183]; [0, 1, 0, +0.0807+0.0893]; [0, 0, 1, -0.0291]; [0, 0, 0, 1]];

    S = [S1 S2 S3 S4 S5 S6 S7];
    pose_L = FKinSpace(M, S, theta_mini_L);
    Jst_L = JacobianSpace(S,theta_mini_L);
    
end
