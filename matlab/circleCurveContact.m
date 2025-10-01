function [x_contact, y_contact, delta, n_circle, contact_state] = ...
    circleCurveContact(xc, yc, R, X_curve, Y_curve)
% circleCurveContact
% Fast circle–polyline intersection & contact-point extractor.
%
% Inputs
%   xc, yc   : circle center
%   R        : circle radius (>0)
%   X_curve  : 1xN or Nx1 array of curve X vertices (polyline)
%   Y_curve  : 1xN or Nx1 array of curve Y vertices (polyline)
%
% Outputs
%   x_contact, y_contact : contact point (midpoint of chosen intersection pair)
%   delta                : penetration (R - ||[x_contact;y_contact] - center||, >=0)
%   n_circle             : 1x2 inward unit normal of circle at contact point
%   contact_state        : 1 if contact (>=1 intersection), 0 otherwise
%
% Notes
%   - If more than two intersections exist, we pick the pair whose midpoint
%     has the **largest penetration** (most central chord).
%   - If exactly one (tangent) intersection exists, that point is used (delta≈0).
%   - If none, contact_state=0 and outputs are zeros.

    % --- init defaults
    x_contact = 0; y_contact = 0; delta = 0; n_circle = [0, 1]; contact_state = 0;

    % Ensure column vectors
    X_curve = X_curve(:); Y_curve = Y_curve(:);
    N = numel(X_curve);
    if N < 2 || R <= 0
        return;
    end

    % --- quick bounding-box reject (cheap)
    xmin = min(X_curve); xmax = max(X_curve);
    ymin = min(Y_curve); ymax = max(Y_curve);
    if (xmax < xc - R) || (xmin > xc + R) || (ymax < yc - R) || (ymin > yc + R)
        return; % no overlap
    end

    % --- segment endpoints and directions
    Ax = X_curve(1:end-1);  Ay = Y_curve(1:end-1);
    Bx = X_curve(2:end);    By = Y_curve(2:end);
    dx = Bx - Ax;           dy = By - Ay;

    % Quadratic coefficients for |A + t*d - C|^2 = R^2 ; t in [0,1]
    %   a t^2 + b t + c = 0
    a = dx.^2 + dy.^2;  % segment length squared
    % Guard against degenerate segments to avoid division by ~0:
    valid = a > 1e-16;
    if ~any(valid)
        return;
    end
    Ax = Ax(valid); Ay = Ay(valid);
    dx = dx(valid); dy = dy(valid);
    Bx = Ax + dx;   By = Ay + dy;
    a  = a(valid);

    % Shifted by center C = (xc, yc)
    fx = Ax - xc;   fy = Ay - yc;

    b = 2*(dx.*fx + dy.*fy);
    c = (fx.^2 + fy.^2) - R^2;

    % Discriminant
    disc = b.^2 - 4.*a.*c;

    % Keep segments with real intersection
    hasReal = disc >= 0;
    if ~any(hasReal)
        return;
    end

    a  = a(hasReal);  b = b(hasReal);  c = c(hasReal);
    Ax = Ax(hasReal); Ay = Ay(hasReal);
    dx = dx(hasReal); dy = dy(hasReal);

    % Roots t1,t2 (vectorized)
    sqrtDisc = sqrt(max(disc(hasReal), 0));
    inv2a    = 0.5 ./ a;
    t1 = (-b - sqrtDisc) .* inv2a;
    t2 = (-b + sqrtDisc) .* inv2a;

    % Collect roots that lie within [0,1] (with small tolerance)
    tol = 1e-12;
    mask1 = (t1 >= -tol) & (t1 <= 1+tol);
    mask2 = (t2 >= -tol) & (t2 <= 1+tol);

    % Intersection points for valid roots
    P = [];  % Nx2 points
    if any(mask1)
        tt = min(max(t1(mask1), 0), 1); % clamp to [0,1]
        Px = Ax(mask1) + tt .* dx(mask1);
        Py = Ay(mask1) + tt .* dy(mask1);
        P = [P; [Px, Py]];
    end
    if any(mask2)
        tt = min(max(t2(mask2), 0), 1);
        Px = Ax(mask2) + tt .* dx(mask2);
        Py = Ay(mask2) + tt .* dy(mask2);
        P = [P; [Px, Py]];
    end

    if isempty(P)
        return;
    end

    % Remove duplicates (can occur for tangency or overlapping numerics)
    if size(P,1) > 1
        [P, ~] = uniquetol(P, 1e-10, 'ByRows', true);
    end

    % --- choose contact point
    if size(P,1) == 1
        % Tangent case: single intersection
        M = P(1,:); % point on circle
    else
        % More than one intersection: pick pair with **max penetration**
        % i.e., midpoint deepest inside circle (largest R - ||M - C||).
        num = size(P,1);
        bestDelta = -inf;
        bestM = [0,0];

        % All pairs i<j (usually small number)
        for i = 1:num-1
            Pi = P(i,:);
            for j = i+1:num
                Pj = P(j,:);
                Mij = 0.5*(Pi + Pj);            % chord midpoint
                d   = hypot(Mij(1)-xc, Mij(2)-yc);
                delt = max(0, R - d);           % penetration
                if delt > bestDelta
                    bestDelta = delt;
                    bestM = Mij;
                end
            end
        end
        M = bestM;
    end

    % Outputs
    x_contact = M(1);
    y_contact = M(2);
    dC = [xc - x_contact, yc - y_contact];
    nrm = hypot(dC(1), dC(2));
    if nrm < 1e-15
        % Midpoint numerically at center: pick an arbitrary inward normal
        n_circle = [0, 1];
        delta    = R;   % fully 'inside'
    else
        n_circle = dC / nrm;          % inward unit normal (toward center)
        delta    = max(0, R - nrm);   % penetration depth
    end
    contact_state = 1;
end
