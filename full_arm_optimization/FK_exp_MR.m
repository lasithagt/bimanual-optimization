
function [Slist, M1] = FK_exp_MR(x, g_st)
    
    global input
    
    %% manipulator twists
    d1 = 0.0; d2 = x(3); d3 = x(3) + x(4);
    Q = g_st(1:3,end);
    R = g_st(1:3,1:3);
    W = g_st(1:3,1:3) * [0 0 1]';
    w1 = W; q1 = Q;
    w2 = R * [0;-1;0]; q2 = Q + R * [0;0;d1];
    w3 = R * [0;0;1];  q3 = Q + R * [0;0;d1];
    w4 = R * [0;1;0];  q4 = Q + R * [0;0;d2];
    w5 = R * [0;1;0];  q5 = Q + R * [0;0;d3];
    w6 = R * [-1;0;0]; q6 = Q + R * [0;0;d3];
    w7 = R * [0;0;1];  q7 = Q + R * [0;0;d3+input.tool];
    
    
    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);
    
    Slist  = [S1 S2 S3 S4 S5 S6 S7];

    M1 = g_st * [0 1 0 0;-1 0 0 0;0 0 1 d3;0 0 0 1];
    
end
