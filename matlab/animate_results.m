function [] = animate_results(t, y, P, road, fps, size_factor, animate)

% Set size factor if not defined
if nargin < 6; size_factor = 2; end
if nargin < 7; animate = 0; end

% Create full screen figure & set background color to white
figure('units','normalized','outerposition',[0 0 1 1], 'Color', 'w'); 

% Number of frames
nf = floor(t(end) * fps);

% Animation time
t_anim = linspace(0, t(end), nf);

% Initialize frame
frame = {};

% Loop over all frames
for i = 1 : nf

    % Clear figure
    clf;
    hold on; axis equal; box on;

    % Get y_i
    y_i = (interp1(t,y,t_anim(i)))';

    % Plot car
    PlotHalfCar(y_i(1:5), P);

    % Road coordinates
    x_road = linspace(y_i(1)-size_factor*(P.lf + P.lr), y_i(1)+size_factor*(P.lf + P.lr), size_factor*200);
    y_road = road.h(x_road);
    plot(x_road, y_road, 'k','LineWidth',3);

    % Remove axis ticks and labels
    set(gca, 'XTick', [], 'YTick', []);
    set(gca, 'XTickLabel', [], 'YTickLabel', []);

    % Add title
    if animate == 0; title('Animation of the downhill run', 'FontSize', 14, 'FontWeight', 'bold'); end

    % Display time, distance, and velocity in upper right corner
    time_str = sprintf('t = %.2f s', t_anim(i));
    distance_str = sprintf('d = %.0f m', y_i(1));
    velocity = sqrt(y_i(6)^2 + y_i(7)^2) * 3.6;  % Convert m/s to km/h
    velocity_str = sprintf('V = %.1f km/h', velocity);

    % Get axis limits for positioning
    xlim_current = xlim;
    ylim_current = ylim;
    
    % Position text in upper right corner
    text_x = xlim_current(2) - 0.05 * (xlim_current(2) - xlim_current(1));
    text_y = ylim_current(2) - 0.05 * (ylim_current(2) - ylim_current(1));
    
    text(text_x, text_y, {time_str; distance_str; velocity_str}, ...
         'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
         'FontSize', 12, 'FontWeight', 'bold', ...
         'BackgroundColor', 'white', 'EdgeColor', 'black', 'Margin', 2);

    drawnow;

    % Save frame
    if animate
        fig = gcf;
        currentFrame = getframe(fig);
        frame{end+1} = currentFrame;
    end

end

% Generate mp4 & gif
if animate
    SaveFrames(frame, pwd, 1/fps, 1, 'downhill_mtb');
end





