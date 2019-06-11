% Gives forward kinemtaics of the whole dual arm
function fwk = FK(x, q)
    
    global input
    
    fwk = zeros(4,4,2);
    fwk(:,:,1) = eye(4); fwk(:,:,2) = eye(4); 
    
    % compute the left robot fk_L rigt robot fk_R
    T_L = input.T_L;
    T_R = input.T_R;
     
    %     T_L(1:3,1:3) = roty(pi/2);
    %     T_L(1:3,end) = [x(1);0;0];
    %     T_R(1:3,1:3) = roty(-pi/2);
    %     T_R(1:3,end) = [-x(1);0;0];
    
    q_L = q(1,:);  % First row of the q vector is Left arm
    q_R = q(2,:);  % Second row of the q vector is Right arm

    fk_L = T_L * FK_func(q_L, x(5:end));
    fk_R = T_R * FK_func(q_R, x(5:end));
    
%     fk_L = FK_func(q_L, x(5:end));
%     fk_R = FK_func(q_R, x(5:end));
    
    fwk(:,:,1) = fk_L; fwk(:,:,2) = fk_R;
    
end

function fk = FK_func(q_,x)

    d1 = x(1); d2 = x(2); d3 = x(3); d4 = 0; tool = 1;
    a1 = 0; a2 = 0; a3 = 0; a4 = x(4);

    q = @(n)q_(n+1);

    FK(1) = ((((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*sin(q(3)) - sin(q(1))*cos(q(0))*cos(q(3)))*cos(q(4)) + ((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(3)) + sin(q(1))*sin(q(3))*cos(q(0)))*sin(q(4)))*sin(q(5)) - (-sin(q(0))*cos(q(2)) - sin(q(2))*cos(q(0))*cos(q(1)))*cos(q(5)))*cos(q(6)) + (-((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*sin(q(3)) - sin(q(1))*cos(q(0))*cos(q(3)))*sin(q(4)) + ((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(3)) + sin(q(1))*sin(q(3))*cos(q(0)))*cos(q(4)))*sin(q(6));
    FK(2) = -((((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*sin(q(3)) - sin(q(1))*cos(q(0))*cos(q(3)))*cos(q(4)) + ((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(3)) + sin(q(1))*sin(q(3))*cos(q(0)))*sin(q(4)))*sin(q(5)) - (-sin(q(0))*cos(q(2)) - sin(q(2))*cos(q(0))*cos(q(1)))*cos(q(5)))*sin(q(6)) + (-((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*sin(q(3)) - sin(q(1))*cos(q(0))*cos(q(3)))*sin(q(4)) + ((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(3)) + sin(q(1))*sin(q(3))*cos(q(0)))*cos(q(4)))*cos(q(6));
    FK(3) = (((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*sin(q(3)) - sin(q(1))*cos(q(0))*cos(q(3)))*cos(q(4)) + ((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(3)) + sin(q(1))*sin(q(3))*cos(q(0)))*sin(q(4)))*cos(q(5)) + (-sin(q(0))*cos(q(2)) - sin(q(2))*cos(q(0))*cos(q(1)))*sin(q(5));
    FK(4) = a1*cos(q(0)) + a2*cos(q(0))*cos(q(1)) - a3*sin(q(0))*sin(q(2)) + a3*cos(q(0))*cos(q(1))*cos(q(2)) + a4*(-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*sin(q(3)) - a4*sin(q(1))*cos(q(0))*cos(q(3)) + d2*sin(q(0)) - d3*sin(q(1))*cos(q(0)) + d4*(-sin(q(0))*cos(q(2)) - sin(q(2))*cos(q(0))*cos(q(1))) + tool*((((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*sin(q(3)) - sin(q(1))*cos(q(0))*cos(q(3)))*cos(q(4)) + ((-sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(3)) + sin(q(1))*sin(q(3))*cos(q(0)))*sin(q(4)))*cos(q(5)) + (-sin(q(0))*cos(q(2)) - sin(q(2))*cos(q(0))*cos(q(1)))*sin(q(5)));
    FK(5) = ((((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*sin(q(1))*cos(q(3)))*cos(q(4)) + ((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(1))*sin(q(3)))*sin(q(4)))*sin(q(5)) - (-sin(q(0))*sin(q(2))*cos(q(1)) + cos(q(0))*cos(q(2)))*cos(q(5)))*cos(q(6)) + (-((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*sin(q(1))*cos(q(3)))*sin(q(4)) + ((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(1))*sin(q(3)))*cos(q(4)))*sin(q(6));
    FK(6) = -((((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*sin(q(1))*cos(q(3)))*cos(q(4)) + ((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(1))*sin(q(3)))*sin(q(4)))*sin(q(5)) - (-sin(q(0))*sin(q(2))*cos(q(1)) + cos(q(0))*cos(q(2)))*cos(q(5)))*sin(q(6)) + (-((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*sin(q(1))*cos(q(3)))*sin(q(4)) + ((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(1))*sin(q(3)))*cos(q(4)))*cos(q(6));
    FK(7) = (((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*sin(q(1))*cos(q(3)))*cos(q(4)) + ((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(1))*sin(q(3)))*sin(q(4)))*cos(q(5)) + (-sin(q(0))*sin(q(2))*cos(q(1)) + cos(q(0))*cos(q(2)))*sin(q(5));
    FK(8) = a1*sin(q(0)) + a2*sin(q(0))*cos(q(1)) + a3*sin(q(0))*cos(q(1))*cos(q(2)) + a3*sin(q(2))*cos(q(0)) + a4*(sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - a4*sin(q(0))*sin(q(1))*cos(q(3)) - d2*cos(q(0)) - d3*sin(q(0))*sin(q(1)) + d4*(-sin(q(0))*sin(q(2))*cos(q(1)) + cos(q(0))*cos(q(2))) + tool*((((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*sin(q(1))*cos(q(3)))*cos(q(4)) + ((sin(q(0))*cos(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(1))*sin(q(3)))*sin(q(4)))*cos(q(5)) + (-sin(q(0))*sin(q(2))*cos(q(1)) + cos(q(0))*cos(q(2)))*sin(q(5)));
    FK(9) = (((sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3)))*cos(q(4)) + (sin(q(1))*cos(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(4)))*sin(q(5)) + sin(q(1))*sin(q(2))*cos(q(5)))*cos(q(6)) + (-(sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3)))*sin(q(4)) + (sin(q(1))*cos(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)))*sin(q(6));
    FK(10) = -(((sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3)))*cos(q(4)) + (sin(q(1))*cos(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(4)))*sin(q(5)) + sin(q(1))*sin(q(2))*cos(q(5)))*sin(q(6)) + (-(sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3)))*sin(q(4)) + (sin(q(1))*cos(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)))*cos(q(6));
    FK(11) = ((sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3)))*cos(q(4)) + (sin(q(1))*cos(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(4)))*cos(q(5)) - sin(q(1))*sin(q(2))*sin(q(5));
    FK(12) = a2*sin(q(1)) + a3*sin(q(1))*cos(q(2)) + a4*sin(q(1))*sin(q(3))*cos(q(2)) + a4*cos(q(1))*cos(q(3)) + d1 + d3*cos(q(1)) - d4*sin(q(1))*sin(q(2)) + tool*(((sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3)))*cos(q(4)) + (sin(q(1))*cos(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(4)))*cos(q(5)) - sin(q(1))*sin(q(2))*sin(q(5)));
    FK(13) = 0;
    FK(14) = 0;
    FK(15) = 0;
    FK(16) = 1;

    fk = reshape(FK,[],4)';

end