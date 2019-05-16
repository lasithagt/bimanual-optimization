function [fk_u, fk_uu] = FK_u(lq)
    q1 = lq(1); q2 = lq(2); q3 = lq(3); q4 = lq(4); l1=lq(5); l2=lq(6); l3=lq(7); l4=lq(8);
    scew_x = @(x)[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0];

    
    fwk_q1 = [-l4*sin(q1 + q2 + q3 + q4) - l2*sin(q1 + q2) - l1*sin(q1) - l3*sin(q1 + q2 + q3);...
                l4*cos(q1 + q2 + q3 + q4) + l2*cos(q1 + q2) + l1*cos(q1) + l3*cos(q1 + q2 + q3);0];
    fwk_q2 = [-l4*sin(q1 + q2 + q3 + q4) - l2*sin(q1 + q2) - l3*sin(q1 + q2 + q3);...
                l4*cos(q1 + q2 + q3 + q4) + l2*cos(q1 + q2) + l3*cos(q1 + q2 + q3);0];
    fwk_q3 = [-l4*sin(q1 + q2 + q3 + q4) - l3*sin(q1 + q2 + q3);...
                l4*cos(q1 + q2 + q3 + q4) + l3*cos(q1 + q2 + q3);0];
    fwk_q4 = [-l4*sin(q1 + q2 + q3 + q4);l4*cos(q1 + q2 + q3 + q4);0];
    fwk_l1 = [cos(q1);sin(q1);0];
    fwk_l2 = [cos(q1 + q2);sin(q1 + q2);0];
    fwk_l3 = [cos(q1 + q2 + q3);sin(q1 + q2 + q3);0];
    fwk_l4 = [cos(q1 + q2 + q3 + q4);sin(q1 + q2 + q3 + q4);0];
    
    
    fwk_q1_q1 =  [-l4*cos(q1 + q2 + q3 + q4) - l2*cos(q1 + q2) - l1*cos(q1) - l3*cos(q1 + q2 + q3);...
            -l4*sin(q1 + q2 + q3 + q4) - l2*sin(q1 + q2) - l1*sin(q1) - l3*sin(q1 + q2 + q3);0];                                                             
    fwk_q1_q2 =  [-l4*cos(q1 + q2 + q3 + q4) - l2*cos(q1 + q2) - l3*cos(q1 + q2 + q3);...
        - l4*sin(q1 + q2 + q3 + q4) - l2*sin(q1 + q2) - l3*sin(q1 + q2 + q3);0];
    fwk_q1_q3 =  [-l4*cos(q1 + q2 + q3 + q4) - l3*cos(q1 + q2 + q3);...
            -l4*sin(q1 + q2 + q3 + q4) - l3*sin(q1 + q2 + q3);0];
    fwk_q1_q4 =  [-l4*cos(q1 + q2 + q3 + q4);-l4*sin(q1 + q2 + q3 + q4);0];
    fwk_q1_l1 = [-sin(q1);cos(q1);0];
    fwk_q1_l2 = [-sin(q1 + q2);cos(q1 + q2);0];
    fwk_q1_l3 = [-sin(q1 + q2 + q3);cos(q1 + q2 + q3);0];
    fwk_q1_l4 = [-sin(q1 + q2 + q3 + q4);cos(q1 + q2 + q3 + q4);0];
    
    
    fwk_q2_q2 =  [-l4*cos(q1 + q2 + q3 + q4) - l2*cos(q1 + q2) - l3*cos(q1 + q2 + q3);...
                - l4*sin(q1 + q2 + q3 + q4) - l2*sin(q1 + q2) - l3*sin(q1 + q2 + q3);0];
    fwk_q2_q3 =  [-l4*cos(q1 + q2 + q3 + q4) - l3*cos(q1 + q2 + q3);...
                - l4*sin(q1 + q2 + q3 + q4) - l3*sin(q1 + q2 + q3);0];
    fwk_q2_q4 = [-l4*cos(q1 + q2 + q3 + q4);-l4*sin(q1 + q2 + q3 + q4);0];
    fwk_q2_l1 = [0;0;0];
    fwk_q2_l2 = [-sin(q1 + q2);cos(q1 + q2);0];
    fwk_q2_l3 = [-sin(q1 + q2 + q3);cos(q1 + q2 + q3);0];
    fwk_q2_l4 = [-sin(q1 + q2 + q3 + q4);cos(q1 + q2 + q3 + q4);0];
    
    fwk_q3_q3 = [-l4*cos(q1 + q2 + q3 + q4) - l3*cos(q1 + q2 + q3);...
                - l4*sin(q1 + q2 + q3 + q4) - l3*sin(q1 + q2 + q3);0];
    fwk_q3_q4 = [-l4*cos(q1 + q2 + q3 + q4);...
                -l4*sin(q1 + q2 + q3 + q4);0];
    fwk_q3_l1 = [0;0;0];
    fwk_q3_l2 = [0;0;0];
    fwk_q3_l3 = [-sin(q1 + q2 + q3);cos(q1 + q2 + q3);0];
    fwk_q3_l4 = [-sin(q1 + q2 + q3 + q4);cos(q1 + q2 + q3 + q4);0];

    fwk_q4_q4 = [-l4*cos(q1 + q2 + q3 + q4);-l4*sin(q1 + q2 + q3 + q4);0];
    fwk_q4_l1 = [0;0;0];
    fwk_q4_l2 = [0;0;0];
    fwk_q4_l3 = [0;0;0];
    fwk_q4_l4 = [-sin(q1 + q2 + q3 + q4);cos(q1 + q2 + q3 + q4);0];
    
    fwk_l1_l1 = [0;0;0];
    fwk_l1_l2 = [0;0;0];
    fwk_l1_l3 = [0;0;0];
    fwk_l1_l4 = [0;0;0];
    
    fwk_l2_l2 = [0;0;0];
    fwk_l2_l3 = [0;0;0];
    fwk_l2_l4 = [0;0;0];
    
    fwk_l3_l3 = [0;0;0];
    fwk_l3_l4 = [0;0;0];
    fwk_l4_l4 = [0;0;0];
        
    % Jacobian [7 x 8]
    R  = FK(lq);
    R  = R(1:3,1:3);
    q  = rotm2quat(R); % Convert to Quaternions
    dq_dz = @(w,q)[0.5 0.5 0.5 0.5 0 0 0 0;zeros(2,8);[ones(1,4)*cos(w/2) zeros(1,4)]];
    fk_u  = [fwk_q1 fwk_q2 fwk_q3 fwk_q4 fwk_l1 fwk_l2 fwk_l3 fwk_l4;dq_dz(q(1),q(2:end))];
    
    % Hessian [7 x 8 x 8] Tensor
    fk_uu = zeros(7,8,8);
    dq_dzz = @(w,q)[0 0 0 0 0 0 0 0;zeros(2,8);[-0.5*ones(1,4)*sin(w/2) zeros(1,4)]];
    
    fk_uu(:,:,1) = [fwk_q1_q1 fwk_q1_q2 fwk_q1_q3 fwk_q1_q4 fwk_q1_l1 fwk_q1_l2 fwk_q1_l3 fwk_q1_l4;dq_dzz(q(1),q(2:end))];
    fk_uu(:,:,2) = [fwk_q1_q2 fwk_q2_q2 fwk_q2_q3 fwk_q2_q4 fwk_q2_l1 fwk_q2_l2 fwk_q2_l3 fwk_q2_l4;dq_dzz(q(1),q(2:end))];
    fk_uu(:,:,3) = [fwk_q1_q3 fwk_q2_q3 fwk_q3_q3 fwk_q3_q4 fwk_q3_l1 fwk_q3_l2 fwk_q3_l3 fwk_q3_l4;dq_dzz(q(1),q(2:end))];
    fk_uu(:,:,4) = [fwk_q1_q4 fwk_q2_q4 fwk_q3_q4 fwk_q4_q4 fwk_q4_l1 fwk_q4_l2 fwk_q4_l3 fwk_q4_l4;dq_dzz(q(1),q(2:end))];
    fk_uu(:,:,5) = [fwk_q1_l1 fwk_q2_l1 fwk_q3_l1 fwk_q4_l1 fwk_l1_l1 fwk_l1_l2 fwk_l1_l3 fwk_l1_l4;zeros(4,8)];
    fk_uu(:,:,6) = [fwk_q1_l2 fwk_q2_l2 fwk_q3_l2 fwk_q4_l2 fwk_l1_l2 fwk_l2_l2 fwk_l2_l3 fwk_l2_l4;zeros(4,8)];
    fk_uu(:,:,7) = [fwk_q1_l3 fwk_q2_l3 fwk_q3_l3 fwk_q4_l3 fwk_l1_l3 fwk_l2_l3 fwk_l3_l3 fwk_l3_l4;zeros(4,8)];
    fk_uu(:,:,8) = [fwk_q1_l4 fwk_q2_l4 fwk_q3_l4 fwk_q4_l4 fwk_l1_l4 fwk_l2_l4 fwk_l3_l4 fwk_l4_l4;zeros(4,8)];
             
    
end
