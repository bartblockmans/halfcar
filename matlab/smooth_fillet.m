function [X, Y] = smooth_fillet(x1, y1, d1, d2, grad1, grad2, R_curve)
%SMOOTH_FILLET Create a smooth path with a circular arc (fillet) between two lines.
%
%   [X,Y] = smooth_fillet(x1,y1,d1,d2,grad1,grad2,R_curve)
%
% Inputs
%   x1,y1     : start point of the path
%   d1        : length of first straight segment (before the corner)
%   d2        : length of second straight segment (after the corner)
%   grad1     : slope (dy/dx) of the first segment  (use Inf for vertical)
%   grad2     : slope (dy/dx) of the second segment (use Inf for vertical)
%   R_curve   : desired fillet radius (must be > 0)
%
% Outputs
%   X, Y      : polyline coordinates of the smooth curve
%
% Notes
% - The first straight goes from P0 = (x1,y1) in direction grad1 for length d1.
% - The second straight starts at the “knee” point and goes in direction grad2
%   for length d2.
% - The sharp knee is replaced by a circular arc of radius R_curve tangent to both.
% - If the radius is too large to fit given d1/d2, it is reduced to the max feasible.
%
% Author: ChatGPT (GPT-5 Thinking)

    % Basic checks
    if R_curve <= 0
        error('R_curve must be positive.');
    end
    if d1 <= 0 || d2 <= 0
        error('d1 and d2 must be positive.');
    end

    % Helper to convert slope to direction angle
    % (atan handles +/-Inf -> +/-pi/2 automatically)
    th1 = atan(grad1);
    th2 = atan(grad2);

    % Unit direction vectors for the two lines
    u1 = [cos(th1), sin(th1)];
    u2 = [cos(th2), sin(th2)];

    % Start point and knee (sharp corner) before smoothing
    P0    = [x1, y1];
    Pknee = P0 + d1 * u1;     % end of first segment; start of second (sharp)

    % Angle between directions
    dot12 = max(-1,min(1, dot(u1,u2)));
    Delta = acos(dot12);      % included angle in [0,pi]

    % Handle degenerate cases
    epsA = 1e-12;
    if Delta < 1e-12
        % Directions nearly collinear and same direction -> no corner
        % Just straight through: d1 + d2 along u1
        P_end = P0 + (d1 + d2) * u1;
        n1 = max(2, ceil((d1)/0.01));  % sampling based on ~1 cm step (arbitrary)
        n2 = max(2, ceil((d2)/0.01));
        X = [linspace(P0(1), Pknee(1), n1), linspace(Pknee(1), P_end(1), n2)];
        Y = [linspace(P0(2), Pknee(2), n1), linspace(Pknee(2), P_end(2), n2)];
        % Remove possible duplicate at join
        [X, Y] = dedup_join(X, Y);
        return;
    end
    if abs(Delta - pi) < 1e-9
        % Opposite directions: a U-turn corner; tan(Delta/2) -> inf (no finite fillet)
        % Fall back: keep sharp corner (no arc).
        warning('Directions are opposite; cannot place a finite-radius fillet. Returning sharp corner.');
        n1 = max(2, ceil(d1/0.01));
        n2 = max(2, ceil(d2/0.01));
        X = [linspace(P0(1), Pknee(1), n1), linspace(Pknee(1), Pknee(1) + d2*u2(1), n2)];
        Y = [linspace(P0(2), Pknee(2), n1), linspace(Pknee(2), Pknee(2) + d2*u2(2), n2)];
        [X, Y] = dedup_join(X, Y);
        return;
    end

    % Tangent setback distance along each line from the knee
    % t = R * tan(Delta/2)
    t_needed = R_curve * tan(Delta/2);

    % If the setback doesn't fit in d1 or d2, reduce R to the maximum feasible
    t_max = min(d1, d2);
    if t_needed > t_max - 1e-12
        R_old = R_curve;
        t_fit = max(t_max - 1e-9, 0);              % tiny epsilon margin
        if tan(Delta/2) < epsA
            % practically no corner -> already handled, but safeguard
            R_curve = 0;
        else
            R_curve = t_fit / tan(Delta/2);
        end
        if R_curve <= 0
            % No arc fits; return sharp corner
            warning('Fillet radius too large for given d1/d2; returning sharp corner.');
            n1 = max(2, ceil(d1/0.01));
            n2 = max(2, ceil(d2/0.01));
            X = [linspace(P0(1), Pknee(1), n1), linspace(Pknee(1), Pknee(1) + d2*u2(1), n2)];
            Y = [linspace(P0(2), Pknee(2), n1), linspace(Pknee(2), Pknee(2) + d2*u2(2), n2)];
            [X, Y] = dedup_join(X, Y);
            return;
        else
            warning('Fillet radius reduced from %.6g to %.6g to fit d1/d2.', R_old, R_curve);
        end
        t = t_fit;
    else
        t = t_needed;
    end

    % Tangent points (trim along -u1 and +u2 away from knee)
    T1 = Pknee - t * u1;          % end of straight 1
    T2 = Pknee + t * u2;          % start of straight 2

    % Determine turn direction using 2D cross product (z-component)
    z = u1(1)*u2(2) - u1(2)*u2(1);    % >0 => CCW turn, <0 => CW turn
    sgn = sign(z);                     % +1 (CCW) or -1 (CW)

    % Unit normal to u1 (left is [-uy, ux], right is [uy, -ux])
    n1 = sgn * [-u1(2), u1(1)];        % points toward circle center for the arc
    % Center of the fillet circle
    C = T1 + R_curve * n1;

    % For sanity, T2 should satisfy C ≈ T2 + R * (sgn * [-u2y, u2x])
    % (We don't enforce; geometry ensures it when no floating errors.)

    % Build samples
    % Sample straight 1: P0 -> T1
    len1_eff = norm(T1 - P0);
    n1s = max(2, ceil(max(10, len1_eff/0.01)));  % adapt density; min ~10 pts
    X1 = linspace(P0(1), T1(1), n1s);
    Y1 = linspace(P0(2), T1(2), n1s);

    % Sample arc from T1 to T2 around center C
    a1 = atan2(T1(2) - C(2), T1(1) - C(1));
    a2 = atan2(T2(2) - C(2), T2(1) - C(1));

    if sgn > 0
        % CCW: increase angle from a1 to a2
        if a2 <= a1
            a2 = a2 + 2*pi;
        end
        arc_len = R_curve * (a2 - a1);
        nA = max(12, ceil((a2 - a1) / (pi/90)));  % ~2° increments minimum
        A = linspace(a1, a2, nA);
    else
        % CW: decrease angle from a1 to a2
        if a2 >= a1
            a2 = a2 - 2*pi;
        end
        arc_len = R_curve * (a1 - a2);
        nA = max(12, ceil((a1 - a2) / (pi/90)));
        A = linspace(a1, a2, nA);
    end

    Xarc = C(1) + R_curve * cos(A);
    Yarc = C(2) + R_curve * sin(A);

    % Sample straight 2: T2 -> P_end
    P_end = Pknee + (d2 - t) * u2;
    len2_eff = norm(P_end - T2);
    n2s = max(2, ceil(max(10, len2_eff/0.01)));
    X2 = linspace(T2(1), P_end(1), n2s);
    Y2 = linspace(T2(2), P_end(2), n2s);

    % Concatenate and de-duplicate at junctions
    X = [X1, Xarc, X2];
    Y = [Y1, Yarc, Y2];
    [X, Y] = dedup_join(X, Y);
end

function [Xo, Yo] = dedup_join(X, Y)
    % Remove immediate duplicates caused by concatenation
    XY = [X(:), Y(:)];
    keep = true(size(XY,1),1);
    for k = 2:size(XY,1)
        if all(abs(XY(k,:) - XY(k-1,:)) < 1e-12)
            keep(k) = false;
        end
    end
    Xo = X(keep);
    Yo = Y(keep);
end
