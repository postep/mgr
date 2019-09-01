function generate_plot( X )
[r, c] = size(X);
f=figure(1);
clf(f);
hold on;
pos0 = X(:,1);
plot(pos0(1), pos0(2), 'bx');
for i = 1:c-1
    pos0 = X(:,i);
    pos1 = X(:, i+1);
    figure(1);
    plot(pos1(1), pos1(2), 'go');
    figure(1);
    plot([pos0(1), pos1(1)], [pos0(2), pos1(2)], 'm-');
end

axis([-10, 10, -10, 10]);
hold off;

end

