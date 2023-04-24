function plotter(a, b, T)

p = plot3(a(:, 1), a(:, 2), a(:, 3))
p.Marker = '.'
p.LineStyle = "none"

hold on
axis equal

p_D = plot3(b(:,1), b(:,2), b(:,3))
p_D.Marker = '.'
p_D.LineStyle = "none"
p_D.Color = 'red'

for i = 1 : size(a, 1)
    a_temp = transpose(a(i, :));
    a_temp(4) = 1;
    corrected_intermediate = transpose(T * a_temp);
    corrected_intermediate(:, 4) = [];
    corrected(i, :) = corrected_intermediate;
end

p_cor = plot3(corrected(:,1), corrected(:,2), corrected(:,3))
p_cor.Marker = 'o'
p_cor.LineStyle = "none"
p_cor.Color = 'yellow'

end