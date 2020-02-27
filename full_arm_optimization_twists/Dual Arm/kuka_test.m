function [thetalist, dexterity, ratio] = kuka_test(theta_KUKA)

    %% manipulator twists (kuka)
    W1 = [0 0 1]'; W2 = [0 -1 0]';
    w1 = W1; q1 = [0;0;0.2025];
    w2 = W2; q2 = [0;0;0.2025];
    w3 = W1; q3 = [0;0;0.2025];
    w4 = -W2; q4 = [0;0;0.2025+0.42];
    w5 = W1; q5 = [0;0;0.2025+0.42];
    w6 = W2; q6 = [0;0;0.2025+0.42+0.4];
    w7 = W1; q7 = [0;0;0.2025+0.42+0.4+0.126];

    g_st = [1 0 0 0;0 1 0 0;0 0 1 0.2025+0.42+0.4+0.126;0 0 0 1];

    h = 0;
    S1 = ScrewToAxis(q1,w1, h);
    S2 = ScrewToAxis(q2,w2, h);
    S3 = ScrewToAxis(q3,w3, h);
    S4 = ScrewToAxis(q4,w4, h);
    S5 = ScrewToAxis(q5,w5, h);
    S6 = ScrewToAxis(q6,w6, h);
    S7 = ScrewToAxis(q7,w7, h);

    S        = [S1, S2, S3, S4, S5, S6, S7];
    Slist_K  = S;
    pose_K   = FKinSpace(g_st, Slist_K, theta_KUKA);
    eomg     = 01;
    ev       = 0.001;
    
    rng default
    thetalist0 = [0,0,0,0,0,0,0]' + 0.1*rand(7,1);
    
    % path
    r = 0.06; t = linspace(0,2*pi,10);
    x = r*sin(t); y = r*cos(t); z = 0.8*ones(1,numel(t));
    R = eye(3);
    for i = 1:numel(t)
        [thetalist(:,i), success] = IKinSpace_kuka(S, g_st, RpToTrans(R,[x(i),y(i),z(i)]'), thetalist0, eomg, ev);
        success
%         thetalist = mod(thetalist, 2*pi);
        J_K_s                = JacobianSpace(Slist_K,thetalist(:,i)');
        det(J_K_s*J_K_s')
        dexterity(i)            = det(J_K_s(1:3,:)*J_K_s(1:3,:)');
        ratio(i) = max(eig(J_K_s(1:3,:)*J_K_s(1:3,:)')) / min(eig(J_K_s(1:3,:)*J_K_s(1:3,:)'));
        thetalist0 = thetalist(:,i);
    end
%     thetalist
    
end