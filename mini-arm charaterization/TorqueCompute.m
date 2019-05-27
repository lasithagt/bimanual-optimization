% Author: Lasitha Wijayarathne    
%% Calculate the torque output from forward-backward newton-euler recursion
function T = TorqueCompute(robot, manip_chain, q, qD, qDD, base)
    NData = size(q,1);
    [w,wdot,vdot] = forwardRecursion(NData, robot, manip_chain, q, qD, qDD);
    T = BackwardRecursion(w,wdot,vdot,robot,q,manip_chain);

end

 %% This function recursively computes the w, wdot, vdot and v
 % q --> Joint position trajectory matrix [n x n_dof]
 function [w,wdot,vdot] = forwardRecursion(NData, robot, manip_chain, q, qD, qDD)
    % Inverse dynamic with recursive Newton-Euler
    z0   = [0; 0; 1];
    w    = zeros(3,robot.NDOFs,NData);
    wdot = zeros(3,robot.NDOFs,NData);
    vdot = zeros(3,robot.NDOFs,NData);

    for k = 1 : NData 
        q_    = q(k, :);
        qdot  = qD(k, :);
        qddot = qDD(k, :);
    % ---------------------------------------------------------------------
    % Forward recursion
        for i = 1 : robot.NDOFs
            % p_i = -R'*p;    % The inverse of p

            if i > 1
                T = manip_chain.A(i-1,q_); % gives from i-1 to i. see peter corkes's book
                R = T.R;
                p = T.t;

                w(:,i,k) =  R'*(w(:,i-1,k)) + z0.*qdot(i);                          % (eq. 7.107)

                wdot(:,i,k) = R'*(wdot(:,i-1,k)) +  z0.*qddot(i) + ...              % (eq. 7.108)
                    cross(R'*w(:,i-1,k), z0.*qdot(i));

                vdot(:,i,k) = R'*vdot(:,i-1,k) + R'*cross(wdot(:,i-1,k), p) + ...   % (eq. 7.109)
                    R'*cross(w(:,i-1,k), cross(w(:,i-1,k),p));

            else
                w(:,i,k)    =  (z0.*qdot(i));
                wdot(:,i,k) =  (z0.*qddot(i)); % For the first link.
                vdot(:,i,k) =  (zeros(3,1) - robot.grav); % R'*cross(wdot(:,i,k), p_i) + ...
                    % cross(w(:,i,k), cross(w(:,i,k),p_i)); % zeros(3,1) - R'*obj.grav;
            end
        end
    end
 end

 %% This function calculates the the backward iteration of Newton-Euler formulation to compute the torque
 function Q = BackwardRecursion(w, wdot, vdot, robot, q, manip_chain)
    % Dynamic simulation
    % Backward recursion
    NData = size(q,1);
    z0 = [0; 0; 1];
    for k = 1:NData
        for i = robot.NDOFs:-1:1
            T = manip_chain.A(1:i, q(k,:)); % obj.getTransform(obj.q(k,:),0,i); % Transformation from frame to the the base.
            R = T.R;
            
            % vcdot = vdot(:,i,k) - R'*obj.grav + cross(wdot(:,i,k),obj.mC(:,i)) + ...
            % cross(w(:,i,k),cross(w(:,i,k),obj.mC(:,i)));                         % (eq. 7.110)
            vcdot = vdot(:,i,k) + cross(wdot(:,i,k),robot.mC(:,i)) + ...
                cross(w(:,i,k),cross(w(:,i,k),robot.mC(:,i)));

            F  = robot.m(i)*vcdot;                                                     % (eq. 7.111 with gravity)
            II = R*robot.I(:,:,i)*R';
            N  = II*wdot(:,i,k) + cross(w(:,i,k),II*w(:,i,k));                       % (eq. 7.113), in ith frame

            T = manip_chain.A(i, q(k,:));                                            % obj.getTransform(obj.q(k,:),i,i+1);
            Rii = T.R;
            p   = T.t;             
            if i < robot.NDOFs 
                r_iC = (robot.mC(:,i) - p); 
                
                f(:,i) = Rii*f(:,i+1) + F;                                           % This is right, right
                n(:,i) = Rii*n(:,i+1) + cross(Rii*f(:,i+1), r_iC) + ...
                    -cross(f(:,i), robot.mC(:,i)) + N;       
            else
                f(:,i) = F;
                n(:,i) = -cross(f(:,i), robot.mC(:,i)) + N;                           % When i = n (last link) 
            end
            Q(i,k) = n(:,i)'*z0;
        end
    end

 end
