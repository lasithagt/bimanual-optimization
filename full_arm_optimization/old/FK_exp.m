
function rb = FK_exp(x, g_st)
    
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
    
    l1 = createtwist(w1, q1);
    l2 = createtwist(w2, q2);
    l3 = createtwist(w3, q3);
    l4 = createtwist(w4, q4);
    l5 = createtwist(w5, q5);
    l6 = createtwist(w6, q6);
    l7 = createtwist(w7, q7);
    
    M1 = g_st * [0 1 0 0;-1 0 0 0;0 0 1 d3;0 0 0 1];
    rb = robot({l1, l2, l3, l4, l5, l6, l7}, M1);
    
end
