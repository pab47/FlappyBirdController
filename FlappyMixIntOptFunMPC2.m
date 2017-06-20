function [J,exitflag,bcon,tcon,opttime,PosYnew,fval,xnew] = FlappyMixIntOptFunMPC2(TubesFrontP,TubesVOffset,BirdScreenPos,TubesScreenX,BirdSpeedY,n,SecondP,ThirdP,tubespace)
%% Constants and Initial Conditions
% n =  distance to optimize for
tfp = TubesFrontP;
g = .1356; % gravity constant from game
M = 500; % constant for "big M" method
Yinitial = BirdScreenPos(2); % initial y position
PosY = Yinitial*ones(n,1); %initial guess for y positions
SpeedY = 0*ones(n,1); %initial guess for Y speeds

FrontTube1 = TubesScreenX(TubesFrontP)-BirdScreenPos(1)-2; %front of first tube
FrontTube1Pos = FrontTube1 + BirdScreenPos(1);
FrontTube2 = FrontTube1+tubespace;
BTopTube = 128-TubesVOffset(TubesFrontP); %bottom of Top Tube
TBotTube = 177-TubesVOffset(TubesFrontP); %top of bottom tube
TBotTube2 = 177-TubesVOffset(SecondP);
BTopTube2 = 128-TubesVOffset(SecondP);
TBotTube3 = 177-TubesVOffset(ThirdP);
BTopTube3 = 128-TubesVOffset(ThirdP);
endcon = TBotTube3 + (abs(TBotTube3-TBotTube2)/2);
endcontop = BTopTube3 - (abs(BTopTube3-BTopTube2)/2)-5;
tubediff = BTopTube3-BTopTube2;
endconvel = 0;
if tubediff > 0
    endconvel = 1.1;
else
    endconvel = 20;
end

J = 1*ones(n,1); %initial guess for jumps
x0 = [PosY;SpeedY;J]; % vector of initial guesses

%% build A Matrices for linear programming
q = zeros(1,length(x0));
A1 = zeros(n-1,length(q)); %A1 is equality constraint
for i = 1:n-1
    q(i+1) = 1;
    q(i) = -1;
    q(n+i) = -1;
    A1(i,:) = q;
    q = zeros(1,length(x0));
end
A2 = zeros(n-1,length(q)); %A2-A5 are inequality constraints
for i = 1:n-1
    q(n+1+i) = -1;
    q(2*n+i) = M;
    A2(i,:) = q;
    q = zeros(1,length(x0));
end
A3 = zeros(n-1,length(q));
for i = 1:n-1
    q(n+1+i) = 1;
    q(2*n+i) = M;
    A3(i,:) = q;
    q = zeros(1,length(x0));
end
A4 = zeros(n-1,length(q));
for i = 1:n-1
    q(n+1+i) = -1;
    q(n+i) = 1;
    q(2*n+i) = -M;
    A4(i,:) = q;
    q = zeros(1,length(x0));
end
A5 = zeros(n-1,length(q));
for i = 1:n-1
    q(n+1+i) = 1;
    q(n+i) = -1;
    q(2*n+i) = -M;
    A5(i,:) = q;
    q = zeros(1,length(x0));
end
q2 = zeros(1,length(x0)); 
q2(1) = 1; % A vector to constrain initial position
q3 = zeros(1,length(x0));
q3(n+1) = 1; % Start Velocity is end velocity of last run
q4 = zeros(1,length(x0)); 
q4(n) = 1;
q5 = zeros(1,length(x0)); 
q5(2*n) = 1;
Aeq = [A1;q2;q3]; % final A equality matrix
A = [A2;A3;A4;A5]; %final A inequality matrix
%% Build b vectors
A1Size = size(A1);
beq = [zeros(A1Size(1),1);Yinitial;BirdSpeedY]; %final b equality vector
A2Size = size(A2);
% b2-b5: vectors for inequality constraints
b2 = (2.5*ones(A2Size(1),1)+M*ones(A1Size(1),1));
b3 = (-2.5*ones(A2Size(1),1)+M*ones(A1Size(1),1));
b4 = -g*ones(A2Size(1),1);
b5 = -b4;
b = [b2;b3;b4;b5]; %final b inequality vector
%% Additional constraints

tcon = ones(n,1); %top constraint
bcon = 185*ones(n,1); %bottom constraint
if (FrontTube1-7) <= n && (FrontTube1-7) > 0
    L = n-(FrontTube1-7)+1;
    tcon(FrontTube1-7:n) = (BTopTube+10)*ones(L,1);
    bcon(FrontTube1-7:n) = (TBotTube-10)*ones(L,1);
end
if (FrontTube1Pos-7) <= BirdScreenPos(1)
        L = n;
        tcon(1:n) = (BTopTube+10)*ones(L,1);
        bcon(1:n) = (TBotTube-10)*ones(L,1);
end
if (FrontTube1Pos+32) < (n + BirdScreenPos(1))
    L = n-(FrontTube1Pos+32-BirdScreenPos(1));
    tcon(FrontTube1Pos+33-BirdScreenPos(1):n) = ones(L,1);
    bcon(FrontTube1Pos+33-BirdScreenPos(1):n) = 185*ones(L,1);
end
if (FrontTube2-7) <= n
    L = n-(FrontTube2-7)+1;
    tcon(FrontTube2-7:n) = (BTopTube2+10)*ones(L,1);
    bcon(FrontTube2-7:n) = (TBotTube2-10)*ones(L,1);
end
if (FrontTube2+32) < n
    L = n-(FrontTube2+32);
    tcon(FrontTube2+33:n) = ones(L,1);
    bcon(FrontTube2+33:n) = 185*ones(L,1);
end
%bcon(130:144) = 100*ones(15,1);
%lower and upper bounds on states
lb = [tcon;-2.5*ones(n,1);0*ones(n,1)]; %lb=[pos;speed;J(binary)]
ub = [bcon;inf*ones(n,1);ones(n,1)]; %ub=[pos;speed;J(binary)]
intcon = (2*n+1):3*n; % define J vector to be integers
%% Optimization
%cost function: f = sum of J vector = number of jumps
f = [0*ones(1,n),zeros(1,n),ones(1,n)]; 
%optimization using mixed integer linear programming
%options = optimoptions('intlinprog','ObjectiveCutOff',inf);
tic;
[xnew,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
opttime = toc;
J = [xnew(2*n+1:end-1)];
PosYnew = xnew(1:n);
%% Plot Results
% figure(1)
% plot(xnew(1:n))
% hold on
% plot(bcon*ones(1,n),'g')
% plot(tcon*ones(1,n),'g')
% hold off
% ylim([0 199])
% title('PosY')
% axis ij
% figure(2)
% plot(xnew(n+1:2*n))
% title('SpeedY')
% axis ij
% figure(3)
% plot(xnew(2*n+1:3*n))
% title('jumps')