% parallel calculating to improve time consumption ?
% 2022.08.01
clc;clear;
% initial state of subsatellites(pursuer) and target(space debris)
x10=[-0.2 0 0.2 0 0 0];
x20=[-0.2 0 -0.2 0 0 0];
x30=[0.2 0 0.2 0 0 0];
x40=[0.2 0 -0.2 0 0 0];
xT0=[-1 0 -1 0 0.02 0];
%% parameters
m_pur=50;
m_deb=100;
umax_pur=0.02/m_pur;                                        % 20N or 0.02KN 
umax_deb=0.02/m_deb;                                        % assumption: no escape 
iter_tol=10; 
N=10;
deltaT=0.1;
T=150;
N_tol=T/deltaT;
N_move=N_tol-N;                                  % number of rollout 
gama=20;                                         % penalty coefficient
Q=[10*eye(3) zeros(3,3);zeros(3,3) 100*eye(3)];  % I think that variable coefficient is better.
R=10*eye(3);
Lu = -umax_pur*ones(N,3);                           % The Upper and Lower bounds of the control inputs.
Uu =  umax_pur*ones(N,3); 
%% trajectory of debris
uT=zeros(N_tol,3);
xT_tol=discretecw(xT0,uT,deltaT,N_tol);
% trajectory of pursuers
u1=zeros(N,3)+0.001;u2=zeros(N,3)+0.001;u3=zeros(N,3)+0.001;u4=zeros(N,3)+0.001;
x1=discretecw(x10,u1,deltaT,N);x2=discretecw(x20,u2,deltaT,N);x3=discretecw(x30,u3,deltaT,N);x4=discretecw(x40,u4,deltaT,N);
% save actual trajectory & save control
Xs1(1,:)=x1(1,:);Xs2(1,:)=x2(1,:);Xs3(1,:)=x3(1,:);Xs4(1,:)=x4(1,:);
Us1{1}=u1;Us2{1}=u2;Us3{1}=u3;Us4{1}=u4;
options = optimoptions('fmincon','Algorithm','sqp','Display','none');
%% non-cooperative game
for j=1:N_tol-N
    j
    xT=xT_tol(j:j+N-1,:);
    x1k=Xs1(j,:); x2k=Xs2(j,:); x3k=Xs3(j,:); x4k=Xs4(j,:);      
    % for the same time at j, initial state is the same for every
    % iteration
    for iter=1:iter_tol
       u1=Us1{iter}; u2=Us2{iter}; u3=Us3{iter}; u4=Us4{iter};   
       
       COSTFUNe = @(u) OPTIMALP(u,x1k,deltaT,N,x2,x3,x4,xT,Q,R,gama);                               
       [u1opt,fval, exitflag, output] = fmincon (COSTFUNe,u1,[],[],[],[],Lu,Uu,[],options);    
       x1=discretecw(x1k,u1opt,deltaT,N);    % pursuer1 trajectory in iter 1
                                         
       COSTFUNe = @(u) OPTIMALP(u,x2k,deltaT,N,x1,x3,x4,xT,Q,R,gama);   % now x1 is updated by line 41 above                            
       [u2opt,fval, exitflag, output] = fmincon (COSTFUNe,u2,[],[],[],[],Lu,Uu,[],options);    
       x2=discretecw(x2k,u2opt,deltaT,N); 
                                      
       COSTFUNe = @(u) OPTIMALP(u,x3k,deltaT,N,x1,x2,x4,xT,Q,R,gama);          
       [u3opt,fval, exitflag, output] = fmincon (COSTFUNe,u3,[],[],[],[],Lu,Uu,[],options);    
       x3=discretecw(x3k,u3opt,deltaT,N); 
                                        
       COSTFUNe = @(u) OPTIMALP(u,x4k,deltaT,N,x1,x2,x3,xT,Q,R,gama);          
       [u4opt,fval, exitflag, output] = fmincon (COSTFUNe,u4,[],[],[],[],Lu,Uu,[],options);    
       x4=discretecw(x4k,u4opt,deltaT,N);
       
       Us1{iter+1}=u1opt; Us2{iter+1}=u2opt; Us3{iter+1}=u3opt; Us4{iter+1}=u4opt;
    end
       Xs1(j+1,:)=x1(2,:); Xs2(j+1,:)=x2(2,:);  Xs3(j+1,:)=x3(2,:);  Xs4(j+1,:)=x4(2,:);
       Us1f(j,:)=u1opt(1,:); Us2f(j,:)=u2opt(1,:); Us3f(j,:)=u3opt(1,:); Us4f(j,:)=u4opt(1,:);   % final optimal control after iterations
end
