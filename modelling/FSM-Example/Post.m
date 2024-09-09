figure(1)
subplot(2,1,1);
scatter(out.tout, out.q);
xlim([-1 11]);
ylim([-0.1 1.1]);
hold on

subplot(2,1,2);
plot(0:T, out.input);
xlim([-1 11]);
ylim([-0.1 1.1]);
hold on
