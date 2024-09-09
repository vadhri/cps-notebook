%plot the value

figure(1);
clf
subplot(2,1,1);

plot(out.t, out.input);
xlim([-1 11]);
ylim([-10 130]);

legend("input vs time")

grid on

ylabel("input")

subplot(2,1,2);
plot(out.t,out.x);

xlim([-1 11]);
ylim([-1 2]);

legend("q (output)");

grid on 

ylabel("output vs time")