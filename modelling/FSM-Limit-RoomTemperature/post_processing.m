%post
t = out.t;
Temp = out.T;
u = out.u;
j = out.j;

figure(1);
clf;
subplot(2,1,1); 
plotarc(t,j,Temp);

xlabel("Time[sec]");
ylabel('T');

grid on;

legend("T")

subplot(2,1,2); 
plotarc(t,j,u);

xlabel('time[sec]');
ylabel('u');
grid on;



