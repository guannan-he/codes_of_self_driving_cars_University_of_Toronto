function [x,y,phi] = update_s(xk,yk,phik,delta,v,t,L)
x = xk + v * cos(phik) * t;
y = yk + v * sin(phik) * t;
phi = phik + v * tan(delta) / L * t;
end
