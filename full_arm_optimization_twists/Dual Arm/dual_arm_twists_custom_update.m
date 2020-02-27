 
function [ M_K, Slist_K, M_R, Slist_R, M_L, Slist_L] = dual_arm_twists_custom_update(w, q, T_b)

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
%     g_st = [[R;0 0 0],[0;-0.0893;0.2025+0.42+0.4+0.126+0.2086;1]];
    g_st = [[rotx(pi)*rotz(-0);0 0 0],[0;0;0.2025+0.42+0.4+0.126;1]];

    % g_st = [1 0 0 0;0 1 0 -0.0893;0 0 1 0.2025+0.42+0.4+0.126+0.2086;0 0 0 1];
    M_K = g_st;

    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);
    S  = [S1, S2, S3, S4, S5, S6, S7];
    Slist_K = S;

    %% position and orientation of each arm
    W   = [0 0 1]';
    R_l = T_b{1}(1:3,1:3);
    R_r = T_b{2}(1:3,1:3);
    
    Q_l = T_b{1}(1:3,end);
    Q_r = T_b{2}(1:3,end);
    
    d1  = 0; tool = 0.0;
    
    %% mini-arm kinematics, right
    w1 = R_r * W; 
    q1 = Q_r;
    
    w2 = R_r * rotx(pi/2) * [0 0 1]'; 
    q2 = Q_r + R_r * [0;0;d1];
    
    w3 = R_r * [w{1}(1);w{1}(2);w{1}(3)];  
    q3 = Q_r + R_r * [q{1}(1);q{1}(2);q{1}(3)];
    
    w4 = R_r * [w{2}(1);w{2}(2);w{2}(3)]; 
    q4 = Q_r + R_r * [q{2}(1);q{2}(2);q{2}(3)];
    
    w5 = R_r * [w{3}(1);w{3}(2);w{3}(3)]; 
    q5 = Q_r + R_r * [q{3}(1);q{3}(2);q{3}(3)];
    
    w6 = R_r * rotz(pi/2) * [w{3}(1);w{3}(2);w{3}(3)]; 
    q6 = Q_r + R_r * [q{3}(1);q{3}(2);q{3}(3)];
    
    w7 = R_r * rotx(pi/2) * [w{3}(1);w{3}(2);w{3}(3)]; 
    q7 = Q_r + R_r * [q{3}(1);q{3}(2);q{3}(3)+tool];

    h = 0;
    
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);
   
    rot = [0.3172    0.6939    0.6465;0.6190    0.3650   -0.6954;-0.7185    0.6207   -0.3137];  
    M_R = T_b{2}*[[rot;0 0 0] [[q{3}(1);q{3}(2);q{3}(3)+tool];1]];

    % M_R = [[1, 0, 0, 0.1706+0.1055+0.183]; [0, 1, 0, -0.0807-0.0893]; [0, 0, 1, -0.0291]; [0, 0, 0, 1]];

    Slist_R = [S1 S2 S3 S4 S5 S6 S7];

    %% mini-arm kinematics, left
    w1 = R_l * W; 
    q1 = Q_l;
    
    w2 = R_l * rotx(pi/2) * [0 0 1]'; 
    q2 = Q_l + R_l * [0;0;d1];
    
    w3 = R_l * [w{1}(1);w{1}(2);w{1}(3)];  
    q3 = Q_l + R_l * [q{1}(1);q{1}(2);q{1}(3)];
    
    w4 = R_l * [w{2}(1);w{2}(2);w{2}(3)]; 
    q4 = Q_l + R_l * [q{2}(1);q{2}(2);q{2}(3)];
    
    w5 = R_l * [w{3}(1);w{3}(2);w{3}(3)]; 
    q5 = Q_l + R_l * [q{3}(1);q{3}(2);q{3}(3)];
    
    w6 = R_l * rotz(pi/2) * [w{3}(1);w{3}(2);w{3}(3)];  
    q6 = Q_l + R_l * [q{3}(1);q{3}(2);q{3}(3)];
    
    w7 = R_l * rotx(pi/2) * [w{3}(1);w{3}(2);w{3}(3)];  
    q7 = Q_l + R_l * [q{3}(1);q{3}(2);q{3}(3)+tool];

    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);

    rot = [0.3172    0.6939    0.6465;0.6190    0.3650   -0.6954;-0.7185    0.6207   -0.3137];  
    M_L = T_b{1}*[[rot;0 0 0] [[q{3}(1);q{3}(2);q{3}(3)+tool];1]];
    
    % M_L = [[1, 0, 0, 0.1706+0.1055+0.183]; [0, 1, 0, +0.0807+0.0893]; [0, 0, 1, -0.0291]; [0, 0, 0, 1]];

    Slist_L = [S1 S2 S3 S4 S5 S6 S7];
    
end
