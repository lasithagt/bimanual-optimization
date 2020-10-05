
function [fkDH_store] = FK_DH(robot, q)
    global input
    n = size(q, 2);
    fkDH_store = zeros(input.n_links, n);
    
    for i = 1:n
        fkDH_store(:, i) = robot.A(1:7, q(:,i));
    end
    
end
