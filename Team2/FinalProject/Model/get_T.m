function T=get_T(alpha, a, d, theta)
    A = [1          0           0  a; 
         0 cos(alpha) -sin(alpha)  0;
         0 sin(alpha)  cos(alpha)  0;
         0          0           0  1];
     
    B = [cos(theta) -sin(theta)   0  0; 
         sin(theta)  cos(theta)   0  0;
                  0           0   1  d;
                  0           0   0  1];
    T = A* B;
end