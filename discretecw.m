function [xkN] = discretecw(xk,uk,Ts,N)

xk1 = xk;
xkN = zeros(N,6);
for ct=1:N
    df = cwdyna(xk1,uk(ct,:));
    xk1 = xk1 + Ts*df';
    xkN(ct,:)=xk1;   
end