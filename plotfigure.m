%% trajectory of states
plot3(Xs1(:,1),Xs1(:,2),Xs1(:,3));
hold on;
plot3(Xs2(:,1),Xs2(:,2),Xs2(:,3));
hold on;
plot3(Xs3(:,1),Xs3(:,2),Xs3(:,3));
hold on;
plot3(Xs4(:,1),Xs4(:,2),Xs4(:,3));
hold on;
plot3(xT_tol(:,1),xT_tol(:,2),xT_tol(:,3));
%% trajectory of distance error
Xc=0.25*(Xs1+Xs2+Xs3+Xs4);
N_final=N_move+1;
for j=1:N_final
    Xe(j)=norm(Xc(j,:)-xT_tol(j,:));
end
tspan=0:deltaT:(T-N*deltaT);
figure;
plot(tspan,Xe)
%% trajectory of Ji in time k