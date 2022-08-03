function J = OPTIMALP(u,x,deltaT,N,xbr,xcr,xdr,xTr,Q,R,gama)    % performance index function
% xbr,xcr,xdr,xTr:     reference trajectory
% gama:                penalty factor

xk = x;
uk = u;     % length (N,3)                                             
xa=discretecw(xk,uk,deltaT,N);   % the optimized pursuer in the current time
J = 0;
 for ct=1:N  
% accumulate state tracking cost from x(k+1) to x(k+N).
    X=0.25*(xa(ct,:)+xcr(ct,:)+xbr(ct,:)+xdr(ct,:))-xTr(ct,:);
    J = J + X*Q*X'+uk(ct,:)*R*uk(ct,:)';     
 end
% penalty function based on control barrier function :  state in final time
d_ab=norm(xa(end,:)-xbr(end,:));    % distance for ab ac & ad
d_ac=norm(xa(end,:)-xcr(end,:));
d_ad=norm(xa(end,:)-xdr(end,:));
% range of J_pf 0 to 5
J_pf=2*10e5*(log(0.36./(0.36-(d_ab-0.4).^12))+log(0.36./(0.36-(d_ac-0.4).^12))+log(0.36./(0.36-(d_ad-0.4).^12)));

J=J+gama*J_pf;