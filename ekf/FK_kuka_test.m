function FK = FK_kuka_test(q_)
  tic 
  q = @(n)q_(n+1);
  
  FK(1) = -sin(q(0))*sin(q(1)) + cos(q(0))*cos(q(1));
  FK(2) = -sin(q(0))*cos(q(1)) - sin(q(1))*cos(q(0));
  FK(3) = 0;
  FK(4) = -sin(q(0))*sin(q(1)) + cos(q(0))*cos(q(1)) + cos(q(0));
  FK(5) = sin(q(0))*cos(q(1)) + sin(q(1))*cos(q(0));
  FK(6) = -sin(q(0))*sin(q(1)) + cos(q(0))*cos(q(1));
  FK(7) = 0;
  FK(8) = sin(q(0))*cos(q(1)) + sin(q(0)) + sin(q(1))*cos(q(0));
  FK(9) = 0;
  FK(10) = 0;
  FK(11) = 1;
  FK(12) = 2;
  FK(13) = 0;
  FK(14) = 0;
  FK(15) = 0;
  FK(16) = 1;

  FK = reshape(FK, 4,[])';
  toc
  
end
