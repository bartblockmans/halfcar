function [] = PlotFullSpring(x1, y1, x2, y2, R, RATIO, nc, color, LW)

% Check input arguments
if nargin < 7; nc = 5; end
if nargin < 8; color = 'k'; end
if nargin < 9; LW = 1; end

% Create an instance of spring
spr = Spring(R, nc);

% Length
% L = sqrt((x2-x1)^2 + (y2-y1)^2);

% Spring length
% Ls = RATIO * L;

% Leg length
% Ll = (L - Ls)/2;

% Direction vector
n = [x2; y2] - [x1; y1];

% Coordinates
p1 = [x1; y1];
p2 = [x1 + 0.5 * (1-RATIO) * n(1); y1 + 0.5 * (1-RATIO) * n(2)];
p3 = [x2 - 0.5 * (1-RATIO) * n(1); y2 - 0.5 * (1-RATIO) * n(2)];
p4 = [x2; y2];

% Get coordinates of spring
[x_s, y_s] = spr.getSpr(p2, p3);

% Plot spring
plot(x_s, y_s, 'Color', color, 'LineWidth', LW);

% Plot legs
plot([p1(1); p2(1)], [p1(2); p2(2)], 'Color', color, 'LineWidth', LW);
plot([p3(1); p4(1)], [p3(2); p4(2)], 'Color', color, 'LineWidth', LW);
