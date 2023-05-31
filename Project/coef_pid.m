function [k_d, k_p, k_i] = coef_pid(p1,p2,p3)
    k_d = ((p1+p2+p3) - d1)/n1;
    k_p = ((p1*p2+p1*p3+p3*p3) - d2)/n2;
    k_i = ((p1*p2*p3) - d3)/n1;
end