function h = PlotRectangle(x1,y1,x2,y2,w,line_color,face_color,line_width)
% PlotRectangle Draw a straight-edged rectangle between two points.
%   h = PlotRectangle(x1,y1,x2,y2,w,line_color,face_color,line_width)
%   (x1,y1): center of the bottom edge
%   (x2,y2): center of the top edge
%   w      : width (perpendicular to vector from (x1,y1) to (x2,y2))
%
%   Optional:
%   line_color  : edge color (default 'k')
%   face_color  : face color (default 'w')
%   line_width  : edge thickness (default 1.5)
%
%   Returns handle h to the patch.

    % Defaults
    if nargin < 6 || isempty(line_color),  line_color = 'k'; end
    if nargin < 7 || isempty(face_color),  face_color = 'w'; end
    if nargin < 8 || isempty(line_width),  line_width = 1.5; end

    % Endpoints
    p1 = [x1; y1];
    p2 = [x2; y2];

    % Direction and right-hand normal
    v = p2 - p1;
    L = norm(v);
    if L <= eps
        error('PlotRectangle:Degenerate','Points (x1,y1) and (x2,y2) must be distinct.');
    end
    d = v / L;
    n = [d(2); -d(1)];  % right-hand normal

    % Corners (counter-clockwise: bottom-left, bottom-right, top-right, top-left)
    bl = p1 - (w/2)*n;
    br = p1 + (w/2)*n;
    tr = p2 + (w/2)*n;
    tl = p2 - (w/2)*n;

    X = [bl(1) br(1) tr(1) tl(1)];
    Y = [bl(2) br(2) tr(2) tl(2)];

    % Draw
    h = patch(X, Y, face_color, 'EdgeColor', line_color, 'LineWidth', line_width);
end
