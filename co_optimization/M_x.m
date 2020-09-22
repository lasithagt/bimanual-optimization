% This function gives the Mass Matrix in the workspace
function M_mat = M_x(q,l)
    
    global L_1xx L_1xy L_1xz L_1yy L_1yz L_1zz l_1x l_1y l_1z m_1 fv_1 fc_1 L_2xx L_2xy L_2xz L_2yy L_2yz L_2zz l_2x l_2y l_2z m_2 fv_2 fc_2

    q1=q(1); q2=q(2); l1=l(1); l2=l(2);
    
    x0 = cos(q2); x1 = sin(q2); x2 = l2*x0^2 + l2*x1^2;
    x3 = l1*x0 + x2; x4 = l1*x1; x5 = -l_2y; x6 = L_2zz + l_2x*x3 + x4*x5;
    x7 = l_2x + m_2*x3; x8 = l2*x7 + x6;
    
    M_mat = [[L_1zz+ L_2zz + l1*l_1x - l1*l_2y*sin(q2) + l1*(l1*m_1 + l_1x + (l_2x + m_2*(l1*cos(q2) + l2*sin(q2)^2 + l2*cos(q2)^2))*cos(q2) + (l1*m_2*sin(q2) - l_2y)*sin(q2)) + l_2x*(l1*cos(q2) + l2*sin(q2)^2 + l2*cos(q2)^2) + (l_2x + m_2*(l1*cos(q2) + l2*sin(q2)^2 + l2*cos(q2)^2))*(l2*sin(q2)^2 + l2*cos(q2)^2), L_2zz - l1*l_2y*sin(q2) + l2*(l_2x + m_2*(l1*cos(q2) + l2*sin(q2)^2 + l2*cos(q2)^2)) + l_2x*(l1*cos(q2) + l2*sin(q2)^2 + l2*cos(q2)^2)];...
        [L_2zz - l1*l_2y*sin(q2) + l2*(l_2x + m_2*(l1*cos(q2) + l2*sin(q2)^2 + l2*cos(q2)^2)) + l_2x*(l1*cos(q2) + l2*sin(q2)^2 + l2*cos(q2)^2), L_2zz + l2*l_2x + l2*(l2*m_2 + l_2x)]];
    
    if (rcond(J(q,l)) > 0.01)
        M_mat_x = J(q,l)'\M_mat/(J(q,l));
    else
        J_ = J(q,l) + 0.0001*eye(size(J(q,l)));
        M_mat_x = J_'\M_mat/J_;
    end
end