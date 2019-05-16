function M_mat = M_(lq)
    
    global L_1xx L_1xy L_1xz L_1yy L_1yz L_1zz l_1x l_1y l_1z m_1 fv_1 fc_1 L_2xx L_2xy L_2xz L_2yy L_2yz L_2zz l_2x l_2y l_2z m_2 fv_2 fc_2...
        L_3xx L_3xy L_3xz L_3yy L_3yz L_3zz l_3x l_3y l_3z m_3 fv_3 fc_3 L_4xx L_4xy L_4xz L_4yy L_4yz L_4zz l_4x l_4y l_4z m_4 fv_4 fc_4

    q1 = lq(1); q2 = lq(2); q3 = lq(3); q4 = lq(4); l1=lq(5); l2=lq(6); l3=lq(7); l4=lq(8);
   
    x0 = cos(q2);
    x1 = sin(q2);
    x2 = l2*x0^2 + l2*x1^2;
    x3 = l1*x0 + x2;
    x4 = sin(q3);
    x5 = -l_3y;
    x6 = l1*x1;
    x7 = cos(q3);
    x8 = x3*x4 + x6*x7;
    x9 = cos(q4);
    x10 = -l_4y;
    x11 = -x4; 
    x12 = l3*x4^2 + l3*x7^2;
    x13 = x11*x6 + x12 + x3*x7;
    x14 = sin(q4);
    x15 = x13*x14 + x8*x9; 
    x16 = m_4*x15 + x10;
    x17 = -x14;
    x18 = l4*x14^2 + l4*x9^2;
    x19 = x13*x9 + x17*x8 + x18;
    x20 = l_4x + m_4*x19;
    x21 = m_3*x8 + x16*x9 + x17*x20 + x5;
    x22 = l_3x + m_3*x13 + x14*x16 + x20*x9;
    x23 = l_2x + m_2*x3 + x21*x4 + x22*x7;
    x24 = L_4zz + l_4x*x19 + x10*x15;
    x25 = L_3zz + l_3x*x13 + x18*x20 + x24 + x5*x8;
    x26 = -l_2y;
    x27 = L_2zz + l_2x*x3 + x12*x22 + x25 + x26*x6;
    x28 = l2*x23 + x27;
    x29 = l3*x22 + x25; 
    x30 = l4*x20 + x24;
    x31 = l2*x7 + x12;
    x32 = l2*x4;
    x33 = x17*x32 + x18 + x31*x9;
    x34 = x14*x31 + x32*x9;
    x35 = L_4zz + l_4x*x33 + x10*x34;
    x36 = l_4x + m_4*x33;
    x37 = L_3zz + l_3x*x31 + x18*x36 + x32*x5 + x35;
    x38 = m_4*x34 + x10;
    x39 = l_3x + m_3*x31 + x14*x38 + x36*x9;
    x40 = l3*x39 + x37;
    x41 = l4*x36 + x35;
    x42 = l3*x9 + x18;
    x43 = l3*x14;
    x44 = L_4zz + l_4x*x42 + x10*x43;
    x45 = l_4x + m_4*x42; 
    x46 = l4*x45 + x44;
    
    M_mat = [L_1zz + l1*l_1x + l1*(l1*m_1 + l_1x + x0*x23 + x1*(m_2*x6 + x11*x22 + x21*x7 + x26)) + x2*x23 + x27, x28, x29, x30;...
        x28, L_2zz + l2*l_2x + l2*(l2*m_2 + l_2x + x39*x7 + x4*(m_3*x32 + x17*x36 + x38*x9 + x5)) + x12*x39 + x37, x40, x41;...
        x29, x40, L_3zz + l3*l_3x + l3*(l3*m_3 + l_3x + x14*(m_4*x43 + x10) + x45*x9) + x18*x45 + x44, x46;...
        x30, x41, x46, L_4zz + l4*l_4x + l4*(l4*m_4 + l_4x)];

end