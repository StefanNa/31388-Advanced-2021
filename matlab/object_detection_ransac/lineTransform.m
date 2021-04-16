function [alpha_w, r_w] = lineTransform(alpha_l, r_l, x_l, y_l, theta_l)
    alpha_w = alpha_l + theta_l;
    r_w = r_l + x_l*cos(alpha_w) + y_l*sin(alpha_w);
    if (r_w < 0)
        r_w = -r_w;
        alpha_w = alpha_w + pi;
    end
    if alpha_w > pi
        alpha_w = alpha_w - 2*pi;
    end
    if alpha_w < -pi
        alpha_w = alpha_w + 2*pi;
    end
end