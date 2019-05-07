%% This class serves to identify the inertial parameters of a serial manipulator based on ....
% Author : Lasitha Wijayarathne
% Project : Dynamic system ID for rigid body manipulators


% This uses recursive Newton-Euler formlation to get the dynamics

classdef SerialChainCharacterization
   % q_D   --> The desired joint configuration
   % qD_D  --> The desired joint velocities
   % Robot --> robot object.
   
   properties
      q = []; qD = []; qDD = [];
      robot = []
      t = []
      torque_q = []; torque_numerical = []; torque_analytical  = []; torque_RNE = [];
      NDOFs = 0
      NData = 0
      fq = 0;
      inertial_params = [];
      
      % This uses robot's kinematic structure to compute the kinematic
      % parameters.
      frames = [{'iiwa_link_0'}, {'iiwa_link_1'}, {'iiwa_link_2'}, {'iiwa_link_3'}, {'iiwa_link_4'}, {'iiwa_link_5'},...
          {'iiwa_link_6'}, {'iiwa_link_7'}, {'iiwa_link_ee'}];
      
      grav = [0;0;-9.8];
      m = []; I = []; mC = [];
      
   end
   methods
      % Initializer for the class
      function obj = SerialChainCharacterization(time_s, q, Torque, sample_freq)
         if nargin < 4
            error('Not enough input arguments')
         end
         
         lbr = importrobot('lbr820.urdf');
         lbr.DataFormat = 'row';
         lbr.Gravity  = obj.grav;
         obj.robot    = lbr;
         obj.q        = q;
         obj.t        = time_s;
         obj.fq       = sample_freq;
         obj.NDOFs    = size(obj.q,2); % Define the number of degrees of the system.
         obj.NData    = size(obj.q,1); % Define the number of data points.
         
         for i = 2:obj.NDOFs+1
            obj.m(i-1) = obj.robot.Bodies{i}.Mass;
            I = obj.robot.Bodies{i}.Inertia;
            I_M = [I(1) I(6) I(5);I(6) I(2) I(4);I(5) I(4) I(3)];
            obj.I(:,:,i-1) = I_M;
            obj.mC(:,i-1) = obj.robot.Bodies{i}.CenterOfMass;
         end
         
         close all;
         
         % TODO: Should write a line for frame retriving.
         Fs                     = obj.fq;                           
         obj.qD                 = obj.getDerivative(obj.q, Fs);
         obj.qDD                = obj.getDerivative(obj.qD,Fs);
         obj.torque_q           = obj.getFiltered(Torque, obj.fq);        
         
         obj.inertial_params    = [obj.m(1,:);obj.mC(:,:);reshape([obj.I(1,1,:);obj.I(1,2,:);obj.I(1,3,:);obj.I(2,2,:);obj.I(2,3,:);obj.I(3,3,:)],[],7,1)];
         temp                   = [obj.inertial_params(:);zeros(14,1)];
         obj.inertial_params    = reshape(temp, 12, []);
         
         % TODO: might have to do extra to get the frames right
         
        params    = obj.inertial_params(:);
        params_fc = params(end-13:end);
        params    = params(1:70);
        params    = reshape(params, 10,[]);
        
        params    = [params;reshape(params_fc,2,[])];
        % ----------------------------------------------------------------
        % Conversion to the coordinate system.
        for i = 1 : obj.NDOFs
            T   = obj.getTransform([0 0 0 0 0 0 0],i,i+1);
            p   = T(1:3, end);  
            R   = T(1:3,1:3);
            params(2:4,i) = R'*(params(2:4,i) - p);
            
        end
         params(2:4,2) = params(2:4,2) + [0 0 0.2045]';
         params(2:4,4) = params(2:4,4) + [0 0 0.1845]';
         params(2:4,6) = params(2:4,6) + [0 0 0.081]';
         
         % ---------------------------------------------------------------
         obj.inertial_params    = params;
         params_analytical      = KUKA_Inertial_Params(params);
         obj.torque_RNE         = obj.kukaRNE_Torque();
         obj.torque_numerical   = obj.kukaNumerical_Torque();
         obj.torque_analytical  = obj.kukaAnalytical_Torque(params_analytical);
      end
      
      function dataPlot(obj)
        close all
        for j = 1:obj.NDOFs
            figure(j);

            torque        = obj.torque_q(:,j);
            torque_num    = obj.torque_numerical(j,:);
            torque_analyt = obj.torque_analytical(j,:);
            torque_rne    = obj.torque_RNE(j,:);
            
            plot(obj.t(100:end), torque(100:end),...
                obj.t(100:end), torque_num(100:end),...
                obj.t(100:end), torque_analyt(100:end),...
                obj.t(100:end), torque_rne(100:end));
            
            legend('Torque_{Actual}','Torque_{Numerical}' , 'Torque_{Analytical}', 'Torque_{RNE}');

            xlabel('time (s)');
            s = sprintf('jnt\\_%d', j);
            ylabel(s);
            grid on;
        end
      end

      
      %% This funtion filters out the derivative of the velocity to give the. This should be called in the constructor
      % accleration using a derivative filter
      function [v_d] = getDerivative(obj, qd, Fs)

        if nargin < 2
            Fs = 100;
        end
        %         d = designfilt('differentiatorfir','FilterOrder',Nf,'PassbandFrequency',Fpass, ...
        %             'StopbandFrequency',Fstop, ...
        %             'SampleRate',Fs);
        d = designfilt('lowpassfir', ...
            'PassbandFrequency',0.5,'StopbandFrequency',7, ...
            'PassbandRipple',1,'StopbandAttenuation',60, ...
            'DesignMethod','equiripple','SampleRate',Fs);

        v_d = zeros(obj.NData, obj.NDOFs);
        for i = 1:obj.NDOFs
            v_d(2:1:end,i) = filtfilt(d, diff(qd(:,i)))/(1/Fs);
            v_d(1,i) = (v_d(2,i) + v_d(1,i))/2;
        end
      end
      
      
    %% This funtion filters out the derivative of the velocity to give the. This should be called in the constructor
    % accleration using a derivative filter
    function [q_out] = getFiltered(obj, q, Fs)
        if nargin < 2
            Fs = 20;
        end
        passBand = 0.1;
        stopBand = 50;

        d = designfilt('lowpassfir', ...
            'PassbandFrequency', passBand,'StopbandFrequency', stopBand, ...
            'PassbandRipple',1,'StopbandAttenuation',60, ...
            'DesignMethod','equiripple','SampleRate',Fs);

        q_out = zeros(obj.NData, obj.NDOFs);

        for i = 1:obj.NDOFs
            q_out(:,i) = filtfilt(d, q(:,i));
        end

    end            

    %% Outputs the transformation between given two frames in the first
    %  frame.
    function trans = getTransform(obj,configuration,source, target)
        if nargin < 3
            source = target - 1; %if only target frame is specified, by default, output is with respect to the previous frame.
        end
%         if target > obj.NDOFs
%             warning('Target frame exceeds the number of degrees of the manipulator')
%         end
        source = obj.frames{source+1};
        target = obj.frames{target+1};
        trans  = getTransform(obj.robot, configuration,target,source);
        % R = trans(1:3,1:3); s = trans(1:3,end);
    end

    
    %% This function takes in current control and outputs dx on the model
    function tau = kukaNumerical_Torque(obj)
        % States
        tau = zeros(7, obj.NData);
        tic
        for i = 1:obj.NData
            x_pos   = obj.q(i, :)';         % JointPosition
            x_vel   = obj.qD(i, :)';        % JointVelocity
            x_accel = obj.qDD(i, :)';       % JointAcceleration
            
            M = obj.robot.massMatrix(x_pos');
            CX2 = obj.robot.velocityProduct(x_pos', x_vel');
            Grav = obj.robot.gravityTorque(x_pos');
            tau(:, i) = M*x_accel + CX2' + Grav';
%             tau(:, i) = Grav';

        end
        toc
    end
    
    %% This function computes the torque output using the 
    function tau = kukaAnalytical_Torque(obj, params)
        % States
        tau = zeros(obj.NDOFs, obj.NData);
        tic
        for i = 1:obj.NData
            q_pos   = obj.q(i, :)';         % JointPosition
            q_vel   = obj.qD(i, :)';        % JointVelocity
            q_accel = obj.qDD(i, :)';       % JointAcceleration
            
            M    = M_kuka(params, q_pos');
            CX2  = C_kuka(params, q_pos', q_vel');
            Grav = G_kuka(params, q_pos');
            
            % TODO: Add friction model
  
            tau(:, i) = M*q_accel + Grav' + CX2*q_vel; %  INV_kuka(params, q_pos', q_vel', q_accel'); %
        end
        toc
    end
    
    %% This is a test function for analytical accuracy
    function Grav = gravityTest(obj, params)
        for i = 1 : obj.NDOFs
            T   = obj.getTransform([0 0 0 pi/3 0 0 0],i,i+1);
            % Rii = T(1:3, 1:3);
            p   = T(1:3, end);  
            params(2:4,i) = (params(2:4,i) - p);
        end
        Grav = G_kuka(KUKA_Inertial_Params(params), [0 0 0 pi/3 0 0 0]);
        
    end
    
    %% Calculate the torque output from forward-backward newton-euler recursion
    function T = kukaRNE_Torque(obj)
        close all
        tic
        [w,wdot,vdot] = obj.forwardRecursion();
        T = obj.BackwardRecursion(w,wdot,vdot);
        toc
    end

     %% This function recursively computes the w, wdot, vdot and v
     % q --> Joint position trajectory matrix [n x n_dof]
     function [w,wdot,vdot] = forwardRecursion(obj)
        % Inverse dynamic with recursive Newton-Euler

        z0 = [0; 0; 1];
        w = zeros(3,obj.NDOFs,obj.NData);
        wdot = zeros(3,obj.NDOFs,obj.NData);
        vdot = zeros(3,obj.NDOFs,obj.NData);

        for k = 1 : obj.NData 
            q_ = obj.q(k, :);
            qdot = obj.qD(k, :);
            qddot = obj.qDD(k, :);
        % ---------------------------------------------------------------------
        % Forward recursion
            for i = 1 : obj.NDOFs
                T = obj.getTransform(q_,i-1,i);
                R = T(1:3, 1:3);
                p = T(1:3, end);
                % p_i = -R'*p;    % The inverse of p

                if i > 1
                    w(:,i,k) =  R'*(w(:,i-1,k)) + z0.*qdot(i);                          % (eq. 7.107)

                    wdot(:,i,k) = R'*(wdot(:,i-1,k)) +  z0.*qddot(i) + ...              % (eq. 7.108)
                        cross(R'*w(:,i-1,k), z0.*qdot(i));

                    vdot(:,i,k) = R'*vdot(:,i-1,k) + R'*cross(wdot(:,i-1,k), p) + ...   % (eq. 7.109)
                        R'*cross(w(:,i-1,k), cross(w(:,i-1,k),p));
                    
                else
                    w(:,i,k)    =  (z0.*qdot(i));
                    wdot(:,i,k) =  (z0.*qddot(i));                                      % For the first link.
                    vdot(:,i,k) =  zeros(3,1) - obj.grav; % R'*cross(wdot(:,i,k), p_i) + ...
                    % cross(w(:,i,k), cross(w(:,i,k),p_i)); % zeros(3,1) - R'*obj.grav;
                end
            end
        end
     end

     %% This function calculates the the backward iteration of Newton-Euler formulation to compute the torque
     function Q = BackwardRecursion(obj, w, wdot, vdot)
        % Dynamic simulation
        % Backward recursion
  
        z0 = [0; 0; 1];
        for k = 1:obj.NData
            for i = obj.NDOFs:-1:1
                T = obj.getTransform(obj.q(k,:),0,i); % Transformation from frame to the the base.
                R = T(1:3, 1:3);
                
                % vcdot = vdot(:,i,k) - R'*obj.grav + cross(wdot(:,i,k),obj.mC(:,i)) + ...
                %     cross(w(:,i,k),cross(w(:,i,k),obj.mC(:,i)));                         % (eq. 7.110)
                
                vcdot = vdot(:,i,k) + cross(wdot(:,i,k),obj.mC(:,i)) + ...
                    cross(w(:,i,k),cross(w(:,i,k),obj.mC(:,i)));

                F  = obj.m(i)*vcdot;                                                     % (eq. 7.111 with gravity)
                II = R*obj.I(:,:,i)*R';
                N  = II*wdot(:,i,k) + cross(w(:,i,k),II*w(:,i,k));                       % (eq. 7.113), in ith frame

                if i < obj.NDOFs 
                    T   = obj.getTransform(obj.q(k,:),i,i+1);
                    Rii = T(1:3, 1:3);
                    p   = T(1:3, end);  
                    
                    r_iC   = (obj.mC(:,i) - p); 
                    
                    f(:,i) = Rii*f(:,i+1) + F;                                           % This is right, right
                    n(:,i) = Rii*n(:,i+1) + cross(Rii*f(:,i+1), r_iC) + ...
                        -cross(f(:,i), obj.mC(:,i)) + N;       
                else
                    f(:,i) = F;
                    n(:,i) = -cross(f(:,i), obj.mC(:,i)) + N;                            % When i = n (last link) 
                end
                Q(i,k) = n(:,i)'*z0;
            end
        end
            
     end
      
    end
   
end