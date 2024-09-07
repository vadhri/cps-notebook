%Post processing

figure(1)
x = out.x
clf
plot(x(:,1),x(:,2));
hold on
plot(x(1,1),x(1,2), "*");

xlabel("X1")
ylabel("X2")
