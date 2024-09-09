figure(1)

clf 
subplot(2,1,1)
plot(out.time,out.input(:,1), "X", Color="Red", MarkerSize=10)

legend("v(input)")

grid on
ylabel("input")

subplot(2,1,2)
plot(out.time,out.x(:,1), "X", Color="Red", MarkerSize=10)
legend("q(output)")

grid on;

ylabel("output")
