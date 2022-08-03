function dy = cwdyna(x,u)
%% Dynamic parameters  
  R=36000+6371;               
  Mu=3.98603e5;         
  w0 = sqrt(Mu/R^3);  
  
  dy = zeros(6,1);
  A = [0 0 0 1 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1;
       3*w0^2 0 0 0 2*w0 0;
       0 0 0 -2*w0 0 0;
       0 0 -w0^2 0 0 0];
   B = [zeros(3,3);eye(3)];
   

%% Compute dxdt
dy(1:6) = A*x(1:6)' + B*u'; 