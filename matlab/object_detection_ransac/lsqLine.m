function line = lsqLine(edges)
    % line = LSQLINE(edges) extract a line described in (alpha,r)
    % parameters from a set of points
    sums = sum(edges,2);
    squares = edges*edges';
    N = size(edges,2);
    point_bar = sums/N;
    
    tan_y = 2*sums(1)*sums(2) - 2*N*squares(2,1);
    tan_x = sums(1)^2-sums(2)^2-N*squares(1,1)+N*squares(2,2);
    alpha = atan2(tan_y,tan_x)/2;
    r = point_bar(1)*cos(alpha) + point_bar(2)*sin(alpha);
    
    if r < 0
        if alpha < 0
            alpha = alpha + pi;
        else
            alpha = alpha - pi;
        end
        r= r*-1;
    end
    line = [alpha;r];
end