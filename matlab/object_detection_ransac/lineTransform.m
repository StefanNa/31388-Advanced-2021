function [alpha_w, r_w] = lineTransform(alpha_l, r_l, x_l, y_l, theta_l)
    alpha_w = alpha_l + theta_l;
    r_w = r_l + x_l*cos(alpha_w) + y_l*sin(alpha_w);
    r_w_index = r_w < 0;
    
    r_w(r_w_index) = -r_w(r_w_index);
    alpha_w(r_w_index) = alpha_w(r_w_index) + pi;
    
    alpha_w_large = alpha_w > pi;
    alpha_w(alpha_w_large) = alpha_w(alpha_w_large) - 2*pi;
    
    alpha_w_small = alpha_w < -pi;
    alpha_w(alpha_w_small) = alpha_w(alpha_w_small) + 2*pi;
end