function manipulability_optimizer
% Matching of a desired manipulability ellipsoid as the main task (no
% desired position) using the formulation with the manipulability Jacobien
% (Mandel notation).
%

% First run 'startup_rvc' from the robotics toolbox

addpath('./m_fcts/');


%% Auxiliar variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 1E-2;	% Time step
nbIter = 65; % Number of iterations
Km = 3; % Gain for manipulability control in task space

%% Create robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot parameters 
nbDOFs = 4; %Nb of degrees of freedom
armLength = 4; % Links length

% Robot
L1 = Link('d', 0, 'a', armLength, 'alpha', 0);
robot = SerialLink(repmat(L1,nbDOFs,1));
q = sym('q', [1 nbDOFs]);	% Symbolic robot joints
J = robot.jacob0(q'); % Symbolic Jacobian

% Define the desired manipulability
q_Me_d = [pi/16 ; pi/4 ; pi/8 ; -pi/8]; % task 1
% q_Me_d = [pi/2 ; -pi/6; -pi/2 ; -pi/2]; % task 2

J_Me_d = robot.jacob0(q_Me_d); % Current Jacobian
J_Me_d = J_Me_d(1:2,:);
Me_d = (J_Me_d*J_Me_d');
[v_,e] = eig(Me_d);
e_avg = sum(mean(e)); % get the mean value of eigen values
Me_d  = v_*e_avg/v_;
%% Testing Manipulability Transfer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial conditions
q0 = [pi/2 ; -pi/6; -pi/2 ; -pi/2]; % Initial robot configuration task 1

qt    = q0;
it    = 1; % Iterations counter
h1    = [];
gmm_c = [];

% Initial end-effector position (compatibility with 9.X and 10.X versions
% of robotics toolbox)
Htmp = robot.fkine(q0); % Forward Kinematics
if isobject(Htmp) % SE3 object verification
	x0 = Htmp.t(1:2);
else
	x0 = Htmp(1:2,end);
end

% figure('position',[10 10 1000 450],'color',[1 1 1]);

% Main control loop
a = [4 4 4 4];
while( it < nbIter )
	delete(h1);
	lq      = [qt' a]
	Jt      = robot.jacob0(qt); % Current Jacobian
    Jt_full = Jt;
	Jt      = Jt(1:2,:);
	Htmp    = robot.fkine(qt); % Forward Kinematics (needed for plots)
	Me_ct   = (Jt*Jt'); % Current manipulability
	
    Me_track(:,:,it) = Me_ct;
	qt_track(:,it)   = qt;
	
	% Current end-effector position
	if isobject(Htmp) % SE3 object verification
		xt = Htmp.t(1:2);
	else
		xt = Htmp(1:2,end);
	end

	% Compute manipulability Jacobian
	Jm_t = compute_red_manipulability_Jacobian(Jt_full, 1:2);
	
	% Compute desired joint velocities
	M_diff = logmap(Me_d,Me_ct);
    [fk_u, ~] = FK_u(lq);
    
    % Compute q_l
    fk_l  = fk_u(1:2,5:end);
    fk_q  = fk_u(1:2,1:4);
    q_l   = -pinv(fk_l)*fk_q; % from implicit function theorem
    
	dq_dl = pinv(Jm_t)*Km*symmat2vec(M_diff);
	
	% Updating the  position
    % qt = qt + (dq_T1) * dt;
    q  = q + dq_dl*0.01.*[1;1;1;1]
    
	it = it + 1; % Iterations++

end

% % Cost
% figure()
% hold on;
% for it = 1:nbIter-1
% 	cost(it) = norm(logm(Me_d^-.5*Me_track(:,:,it)*Me_d^-.5),'fro');
% end
% plot([1:nbIter-1].*dt, cost, '-','color',[0 0 .7],'Linewidth',3);
% set(gca,'fontsize',14);
% xlim([0 nbIter*dt])
% xlabel('$t$','fontsize',22,'Interpreter','latex'); ylabel('$d$','fontsize',22,'Interpreter','latex');
% 
% end

%% 
function Jm_red = compute_red_manipulability_Jacobian(J, taskVar)
    % Compute the force manipulability Jacobian (symbolic) in the form of a
    % matrix using Mandel notation.

    if nargin < 2
        taskVar = 1:6;
    end

    Jm = compute_manipulability_Jacobian(J);
    Jm_red = [];
    for i = 1:size(Jm,3)
        Jm_red = [Jm_red, symmat2vec(Jm(taskVar,taskVar,i))];
    end

end

function Jm = compute_manipulability_Jacobian(J)
    % Compute the force manipulability Jacobian (symbolic).

    J_grad = compute_joint_derivative_Jacobian(J);

    Jm = tmprod(J_grad,J,2) + tmprod(permute(J_grad,[2,1,3]),J,1);

    % mat_mult = kdlJacToArma(J)*reshape( arma::mat(permDerivJ.memptr(), permDerivJ.n_elem, 1, false), columns, columns*rows);
    % dJtdq_J = arma::cube(mat_mult.memptr(), rows, rows, columns);

end
%% 
function J_grad = compute_joint_derivative_Jacobian(J)
    % Compute the Jacobian derivative w.r.t joint angles (hybrid Jacobian
    % representation).
    % Ref: H. Bruyninck and J. de Schutter, 1996

    nb_rows = size(J,1); % task space dim.
    nb_cols = size(J,2); % joint space dim.
    J_grad = zeros(nb_rows, nb_cols, nb_cols);
    for i = 1:nb_cols
        for j = 1:nb_cols
            J_i = J(:,i);
            J_j = J(:,j);
            if j < i
                J_grad(1:3,i,j) = cross(J_j(4:6,:),J_i(1:3,:));
                J_grad(4:6,i,j) = cross(J_j(4:6,:),J_i(4:6,:));
            elseif j > i
                J_grad(1:3,i,j) = -cross(J_j(1:3,:),J_i(4:6,:));
            else
                J_grad(1:3,i,j) = cross(J_i(4:6,:),J_i(1:3,:));
            end
        end
    end

end

function U = logmap(X,S)
% Logarithm map (SPD manifold)

    N = size(X,3);

    for n = 1:N
    % 	U(:,:,n) = S^.5 * logm(S^-.5 * X(:,:,n) * S^-.5) * S^.5;
    % 	tic
    % 	U(:,:,n) = S * logm(S\X(:,:,n));
    % 	toc
    % 	tic
      [v,d] = eig(S\X(:,:,n));
      U(:,:,n) = S * v*diag(log(diag(d)))*v^-1;
    % 	toc
    end
end

function v = symmat2vec(M)
% Vectorization of a symmetric matrix

    N = size(M,1);
    v = [];

    v = diag(M);
    for n = 1:N-1
      v = [v; sqrt(2).*diag(M,n)]; % Mandel notation
    end
end

function [S,iperm] = tmprod(T,U,mode)
% Mode-n tensor-matrix product

    size_tens = ones(1,mode);
    size_tens(1:ndims(T)) = size(T);
    N = length(size_tens);

    % Compute the complement of the set of modes.
    bits = ones(1,N);
    bits(mode) = 0;
    modec = 1:N;
    modec = modec(logical(bits(modec)));

    % Permutation of the tensor
    perm = [mode modec];
    size_tens = size_tens(perm);
    S = T; 
    if mode ~= 1
        S = permute(S,perm); 
    end

    % n-mode product
    size_tens(1) = size(U,1);
    S = reshape(U*reshape(S,size(S,1),[]),size_tens);

    % Inverse permutation
    iperm(perm(1:N)) = 1:N;
    S = permute(S,iperm); 

end

