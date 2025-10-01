function [] = PlotCircle(x, y, R, ang0, ang1, color, LW, plot_center, elim0, np)

% Check input arguments
if isempty(ang0); ang0 = 0; end
if isempty(ang1); ang1 = 2 * pi; end
if nargin < 6; color = 'k'; end
if nargin < 7; LW = 1.5; end
if nargin < 8; plot_center = 0; end
if nargin < 9; elim0 = 0; end
if nargin < 10; np = 1000; end

% Angle parameter
ang = linspace(ang0,ang1,np);

% Circle coordinates
x_circle = R * cos(ang);
y_circle = R * sin(ang);

% Add origin
x_c = x + x_circle;
y_c = y + y_circle;

% Eliminate all points that have y-values smaller than 0
if elim0
    x_c(y_c<0) = NaN;
    y_c(y_c<0) = NaN;
end

% Plot
if isempty(color)
    plot(x_c, y_c, 'Color', [0 0 0], 'LineWidth', LW);
else
    fill(x_c, y_c, color,'LineWidth',LW);
end

if plot_center; scatter(x, y, 100, color, 'filled'); end