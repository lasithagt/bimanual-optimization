function grad = dynamic_grad(lq, qdd_des, q_des)

    M_u_         = M_u(lq);
    tau_grad     = eye(4);
    M_u_         = reshape(M_u_, [], 4) * qdd_des';
    grad         = [reshape(M_u_,4,[]) tau_grad];
    
end