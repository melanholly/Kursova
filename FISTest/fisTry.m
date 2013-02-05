Ts = 0.1;
% Plant = c2d(zpk([],[-1 -3 -5],1),Ts);
Plant = tf([1 0 ],1,Ts);
Plant.OutputDelay=5;

C0 = pid(1,1,1,'Ts',Ts,'IF','B','DF','B'); % define PID structure
C = pidtune(Plant,C0) % design PID
[Kp Ki Kd] = piddata(C); % obtain PID gains
%%
FIS = newfis('FIS','mamdani','prod','probor','prod','sum');
FIS = addvar(FIS,'input','E',[-10 10]);
FIS = addmf(FIS,'input',1,'Negative','trimf',[-20 -10 0]);
FIS = addmf(FIS,'input',1,'Zero','trimf',[-10 0 10]);
FIS = addmf(FIS,'input',1,'Positive','trimf',[0 10 20]);
FIS = addvar(FIS,'input','CE',[-10 10]);
FIS = addmf(FIS,'input',2,'Negative','trimf',[-20 -10 0]);
FIS = addmf(FIS,'input',2,'Zero','trimf',[-10 0 10]);
FIS = addmf(FIS,'input',2,'Positive','trimf',[0 10 20]);
FIS = addvar(FIS,'output','u',[-20 20]);
FIS = addmf(FIS,'output',1,'LargeNegative','trimf',[-20 -20 -20]);
FIS = addmf(FIS,'output',1,'SmallNegative','trimf',[-10 -10 -10]);
FIS = addmf(FIS,'output',1,'Zero','trimf',[0 0 0]);
FIS = addmf(FIS,'output',1,'SmallPositive','trimf',[10 10 10]);
FIS = addmf(FIS,'output',1,'LargePositive','trimf',[20 20 20]);
ruleList = [1 1 1 1 1;...   % Rule 1
            1 2 2 1 1;...   % Rule 2
            1 3 3 1 1;...   % Rule 3
            2 1 2 1 1;...   % Rule 4
            2 2 3 1 1;...   % Rule 5
            2 3 4 1 1;...   % Rule 6
            3 1 3 1 1;...   % Rule 7
            3 2 4 1 1;...   % Rule 8
            3 3 5 1 1];     % Rule 9
FIS = addrule(FIS,ruleList);
gensurf(FIS);
GE = 10;
GCE = GE*(Kp-sqrt(Kp^2-4*Ki*Kd))/2/Ki;
GCU = Ki/GE;
GU = Kd/GCE;
%% Step 2b: Implement Fuzzy Inference System Using a 2-D Lookup Table

Step = 10; % use 3 break points for both E and CE inputs
E = -10:Step:10;
CE = -10:Step:10;
N = length(E);
LookUpTableData = zeros(N);
for i=1:N
   for j=1:N
      % compute output u for each combination of break points
      LookUpTableData(i,j) = evalfis([E(i) CE(j)],FIS);
   end
end
