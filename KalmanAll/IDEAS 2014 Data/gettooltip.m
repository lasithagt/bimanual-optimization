for i=1:size(pos,1);
    init_orient = makehgtform('zrotate',angs(i,1))*makehgtform('yrotate',angs(i,2))*makehgtform('xrotate',angs(i,3));
    toolpt = makehgtform('translate',pos(i,:))*init_orient*makehgtform('zrotate',dh(1))...
        *makehgtform('yrotate',dh(2))*makehgtform('translate',[dh(3) 0 0])*[0 0 0 1]';%[xx(1,:) 1]';
    tooltp(i,:) = toolpt(1:3)';
end;