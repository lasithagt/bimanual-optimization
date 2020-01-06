
function [Slist, M1] = manipulator_exp(w, q, g_st)
    
    global input
    
    %% manipulator twists
    d1 = 0.0; 
    % d2 = x(3); d3 = x(3) + x(4);
    Q = g_st(1:3,end);
    R = g_st(1:3,1:3);
    W = R * [0 0 1]';
    
    w1 = W; q1 = Q; % this axis is not changing
    w2 = R * [0;-1;0]; q2 = Q + R * [0;0;d1];
    
    
    w3 = R * [w{1}(1);w{1}(2);w{1}(3)];  q3 = Q + R * [q{1}(1);q{1}(2);q{1}(3)];
    w4 = R * [w{2}(1);w{2}(2);w{2}(3)];  q4 = Q + R * [q{2}(1);q{2}(2);q{2}(3)];
    
    % This is the wrist
    %     w5 = R * [0;1;0];  q5 = Q + R * [0;0;d3];
    %     w6 = R * [-1;0;0]; q6 = Q + R * [0;0;d3];
    %     w7 = R * [0;0;1];  q7 = Q + R * [0;0;d3+input.tool];

    w5 = R * [w{3}(1);w{3}(2);w{3}(3)];    q5 = Q + R * [q{3}(1);q{3}(2);q{3}(3)];
    w6 = rotz(pi/2) * w5;                  q6 = Q + R * [q{3}(1);q{3}(2);q{3}(3)];
    w7 = rotx(pi/2) * w5;                  q7 = Q + R * [q{3}(1);q{3}(2);q{3}(3)+input.tool];
    
    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);
    
    % M1 =  g_st * [0 1 0 0;-1 0 0 0;0 0 1 d3;0 0 0 1];
    Slist  = [S1 S2 S3 S4 S5 S6 S7];
%     M_R    = R * rotx(pi/2) * w5;
%     M_R    = Normalize(M_R);
    
%     M_R    = ProjectToSO3(MatrixLog3(VecToso3(M_R')));
    M_R = eye(3);
    M      = RpToTrans(M_R, q{3}');
    M1     = g_st * M;
    
    

end
