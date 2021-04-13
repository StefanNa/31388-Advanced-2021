function line=lsqlinefunction(points)

sumx = sum(points(1,:));

sumy = sum(points(2,:));

sumxsq = points(1,:)*transpose(points(1,:));

sumysq = points(2,:)*transpose(points(2,:));

sumxy = points(1,:)*transpose(points(2,:));
n=length(points)

alphay = (2*sumx*sumy)-(2*n*sumxy);
alphax = (sumx^2-sumy^2-n*sumxsq+n*sumysq);

alpha = atan2(alphay,alphax)
  
r = cos(alpha)*(sumx/n) + sin(alpha)*(sumy/n)
if r<0
    r = abs(r);
    if alpha < 0
      alpha = alpha+pi;
    end
end

alpha = alpha/2
line = [alpha;r]



end