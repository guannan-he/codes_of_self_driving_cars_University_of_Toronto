function [i,alpha] = find_pos(x,y,xk,yk,ld,phik)
[~,r] = size(x);
for j = 1:r
    if xk < x(j)
        break;
    end
end
for i = j:r
    if (x(i) - xk)^2 + (y(i) - yk)^2 > ld^2
        break;
    end
end
alpha = atan((y(i) - yk) / (x(i) - xk)) - phik;
end