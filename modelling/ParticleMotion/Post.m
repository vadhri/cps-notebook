%

xc = out.x(:, 1);
time = out.t;
plot (time,xc);
hold on 

xv = out.x(:,2);
time = out.t;
plot (time,xv);
hold off

legend("Horizontal Position", "Horizontal Velocity");