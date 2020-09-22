f = @(x)[x(1)^2+x(2)^2;cos(x(2));x(1)^2+x(2)^2];

% test the jacobian
jacobian_test = jacobian(f,3,[2,pi/3]')


% test the hessian
hessian_test = hessian(f, 3, [2,pi/3]')

