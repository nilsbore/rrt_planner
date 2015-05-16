

p1 = [-1; 0];
p2 = [-1.5; -0.95];
diff = p1-p2;

n = [0; -1]; % this is the tangent
v = [n(2); -n(1)]; % this is the normal

radius = -0.5*diff'*diff/(diff'*v)

c = p1 + radius*v;

figure
hold on
quiver(p1(1), p1(2), n(1), n(2))
plot(p1(1), p1(2))
plot(p2(1), p2(2))
plot(c(1), c(2))

circle(c, abs(radius), 1000, 'b-');
axis equal
