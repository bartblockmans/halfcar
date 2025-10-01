function [P_road, T_max] = PowerData(P, road)

% Initialize
P_road = road.x_course * 0;
T_max = road.x_course * 0;

% Loop over all rows of power
for i = 1 : size(P.power, 1)

    % Starting distance
    x_start = P.power(i,1);

    % Check start
    if x_start < P.lr; x_start = P.lr; end

    % Index in road.x_course
    [~, ind_start] = min(abs(road.x_course - x_start));

    % Ending distance
    x_end = P.power(i,2);

    % Index in road.x_course
    [~, ind_end] = min(abs(road.x_course - x_end));

    % Add power
    P_road(ind_start:ind_end) = HaverSinePower(P.power(i,3), road.x_course(ind_start:ind_end), 0.5);

    % Add max torque
    T_max(ind_start:ind_end) = (P_road(ind_start:ind_end) / P.power(i,3)) * P.Tmax;

end
end

% Nested function for Haversine-based power profile
function P_haversine = HaverSinePower(P, x, x_trans)

    % Upramp
    x_up = linspace(x(1), x(1) + x_trans, 101);
    P_up = Haversine(x_up(1), x_up(end), 0, P, x_up);

    % Downramp
    x_down = linspace(x(end)-x_trans, x(end), 101);
    P_down = Haversine(x_down(1), x_down(end), P, 0, x_down);

    % Assemble
    x_updown = [x_up, x_down];
    P_updown = [P_up, P_down];

    % Interpolate
    P_haversine = interp1(x_updown, P_updown, x);

end

% Nested function for HaverSine definition
function F = Haversine(t0, t1, F0, F1, t_all)

    % Initialize
    F = zeros(1, length(t_all));

    % Loop over all t
    for j = 1 : length(t_all)
        t = t_all(j);
        if t <= t0
            F(j) = F0;
        elseif t >= t1
            F(j) = F1;
        else
            F(j) = F0 + (F1-F0) * ((F0>F1) - (sin((pi*(t-t0)/(t1-t0)+pi*(F0>F1))/2)^2) * ((F0>F1) + (F0<F1)*(-1)));
        end
    end

end


