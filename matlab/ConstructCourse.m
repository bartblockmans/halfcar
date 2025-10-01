function [x_course, z_course, course_info] = ConstructCourse(segments, varargin)
%CONSTRUCTCOURSE Build a downhill course from multiple segments
%
% SYNTAX:
%   [x_course, z_course, course_info] = ConstructCourse(segments)
%   [x_course, z_course, course_info] = ConstructCourse(segments, 'resolution', res)
%
% INPUTS:
%   segments - Cell array of segment definitions. Each segment is a struct:
%     .type     - 'flat', 'bumpy', 'jump', 'drop', 'staircase', 'trapezium', 'gap_jump', 'rock_garden', 'root_section', 'curve'
%     .distance - Length of segment (m)
%     .gradient - Road gradient (positive = uphill, negative = downhill)
%     .height   - Feature height (m) - for bumpy/jump/drop/staircase/trapezium/gap_jump/rock_garden/root_section segments
%     .frequency - Bumps per meter - for bumpy segments only
%     .step_length - Length of single step (m) - for staircase segments only
%     .num_steps - Number of steps - for staircase segments only
%     .flat_length - Length of flat section (m) - for trapezium segments only
%     .ramp_inclination - Ramp angle in degrees - for trapezium segments only
%     .ramp_length - Length of each ramp (m) - for trapezium segments only
%     .gap_width - Width of gap (m) - for gap_jump segments only
%     .rock_density - Rocks per meter - for rock_garden segments only
%     .root_density - Roots per meter - for root_section segments only
%     .final_gradient - Ending gradient - for curve segments only
%     .R_curve - Radius of circular arc (m) - for curve segments only
%
% OPTIONAL INPUTS:
%   'resolution' - Point density (points per meter), default = 10
%
% OUTPUTS:
%   x_course - X coordinates of course (m)
%   z_course - Z coordinates of course (m) 
%   course_info - Struct with segment boundaries and metadata
%
% EXAMPLE:
%   segments = {
%     struct('type', 'flat', 'distance', 20, 'gradient', 0);
%     struct('type', 'bumpy', 'distance', 15, 'gradient', -0.1, 'height', 0.05, 'frequency', 2);
%     struct('type', 'jump', 'distance', 10, 'gradient', -0.05, 'height', 1.0);
%     struct('type', 'drop', 'distance', 8, 'gradient', -0.2, 'height', 0.8);
%     struct('type', 'staircase', 'distance', 12, 'gradient', -0.1, 'height', 1.5, 'step_length', 0.8, 'num_steps', 5);
%     struct('type', 'trapezium', 'distance', 15, 'gradient', -0.1, 'height', 1.2, 'flat_length', 3.0, 'ramp_inclination', 25);
%     struct('type', 'gap_jump', 'distance', 12, 'gradient', -0.1, 'height', 1.5, 'gap_width', 3.0);
%     struct('type', 'rock_garden', 'distance', 10, 'gradient', -0.15, 'height', 0.3, 'rock_density', 4);
%     struct('type', 'root_section', 'distance', 8, 'gradient', -0.1, 'height', 0.2, 'root_density', 6);
%     struct('type', 'curve', 'distance', 20, 'gradient', -0.1, 'final_gradient', -0.3, 'R_curve', 15);
%     struct('type', 'flat', 'distance', 25, 'gradient', 0);
%   };
%   [x, z, info] = ConstructCourse(segments);

% Parse inputs
p = inputParser;
addRequired(p, 'segments', @iscell);
addParameter(p, 'resolution', 10, @(x) isnumeric(x) && x > 0);
parse(p, segments, varargin{:});

resolution = p.Results.resolution;

% Validate segments
if isempty(segments)
    error('At least one segment must be defined');
end

% Initialize course
x_course = [];
z_course = [];
course_info = struct();
course_info.segments = segments;
course_info.segment_boundaries = [];
course_info.total_distance = 0;

% Process each segment
current_x = 0;
current_z = 0;
current_slope = 0;

for i = 1:length(segments)
    seg = segments{i};
    
    % Validate segment
    if ~isfield(seg, 'type') || ~isfield(seg, 'distance') || ~isfield(seg, 'gradient')
        error('Segment %d must have fields: type, distance, gradient', i);
    end
    
    if seg.distance <= 0
        error('Segment %d distance must be positive', i);
    end
    
    % Generate segment
    [seg_x, seg_z, seg_slope_end] = generate_segment(seg, current_slope, resolution);
    
    % Offset segment to current position
    if i == 1
        % First segment starts at origin
        seg_x = seg_x + current_x;
        seg_z = seg_z + current_z;
    else
        % Subsequent segments start where previous segment ended
        seg_x = seg_x - seg_x(1) + current_x;
        seg_z = seg_z - seg_z(1) + current_z;
    end
    
    % Append to course
    if isempty(x_course)
        x_course = seg_x;
        z_course = seg_z;
    else
        % Ensure C0 continuity (remove duplicate point)
        x_course = [x_course(1:end-1), seg_x];
        z_course = [z_course(1:end-1), seg_z];
    end
    
    % Update position and slope
    current_x = x_course(end);
    current_z = z_course(end);
    current_slope = seg_slope_end;
    
    % Store segment boundary
    course_info.segment_boundaries(i) = current_x;
end

course_info.total_distance = current_x;

% Ensure monotonic x-coordinates
if any(diff(x_course) <= 0)
    error('Course generation failed: non-monotonic x-coordinates');
end

fprintf('Course constructed: %.1f m total, %d segments, %d points\n', ...
        course_info.total_distance, length(segments), length(x_course));

% Plot individual segments for debugging
% plot_segments(segments, resolution);

end

% =========================================================================
%                            SEGMENT GENERATORS
% =========================================================================

function [x, z, slope_end] = generate_segment(seg, slope_start, resolution)
% Generate individual segment based on type

switch lower(seg.type)
    case 'flat'
        [x, z, slope_end] = generate_flat_segment(seg, slope_start, resolution);
        
    case 'bumpy'
        [x, z, slope_end] = generate_bumpy_segment(seg, slope_start, resolution);
        
    case 'jump'
        [x, z, slope_end] = generate_jump_segment(seg, slope_start, resolution);
        
    case 'drop'
        [x, z, slope_end] = generate_drop_segment(seg, slope_start, resolution);
        
    case 'staircase'
        [x, z, slope_end] = generate_staircase_segment(seg, slope_start, resolution);
        
    case 'trapezium'
        [x, z, slope_end] = generate_trapezium_segment(seg, slope_start, resolution);
        
    case 'gap_jump'
        [x, z, slope_end] = generate_gap_jump_segment(seg, slope_start, resolution);
        
    case 'rock_garden'
        [x, z, slope_end] = generate_rock_garden_segment(seg, slope_start, resolution);
        
    case 'root_section'
        [x, z, slope_end] = generate_root_section_segment(seg, slope_start, resolution);
        
    case 'curve'
        [x, z, slope_end] = generate_curve_segment(seg, slope_start, resolution);
        
    otherwise
        error('Unknown segment type: %s. Available types: flat, bumpy, jump, drop, staircase, trapezium, gap_jump, rock_garden, root_section, curve', seg.type);
end

end

function [x, z, slope_end] = generate_flat_segment(seg, slope_start, resolution)
% Generate flat segment with constant gradient

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% First create flat segment (zero gradient)
z_flat = zeros(size(x));

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_bumpy_segment(seg, slope_start, resolution)
% Generate bumpy segment with sinusoidal bumps

if ~isfield(seg, 'height') || ~isfield(seg, 'frequency')
    error('Bumpy segment requires height and frequency fields');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% First create flat segment with bumps (zero gradient)
bump_amplitude = seg.height;
bump_frequency = seg.frequency;
z_flat = bump_amplitude * sin(2 * pi * bump_frequency * x);

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_jump_segment(seg, slope_start, resolution)
% Generate jump segment with smooth ski-jump style jump

if ~isfield(seg, 'height')
    error('Jump segment requires height field');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% Calculate jump length based on height and inclination
if isfield(seg, 'jump_length')
    % User specified jump length
    jump_length = seg.jump_length;
elseif isfield(seg, 'jump_inclination')
    % User specified jump inclination (in degrees)
    jump_inclination_rad = seg.jump_inclination * pi / 180;
    jump_length = seg.height / tan(jump_inclination_rad);
else
    % Default: 10 degree inclination
    jump_inclination_rad = 10 * pi / 180;
    jump_length = seg.height / tan(jump_inclination_rad);
end

% Ensure jump fits within segment
jump_length = min(jump_length, seg.distance * 0.8); % Max 80% of segment

% Position jump in the middle of the segment
jump_start = (seg.distance - jump_length) / 2;
jump_end = jump_start + jump_length;

% Create smooth jump profile using smoothstep function
jump_indices = (x >= jump_start) & (x <= jump_end);
if any(jump_indices)
    jump_x = x(jump_indices) - jump_start;
    t = jump_x / jump_length;  % Normalize to [0,1]
    
    % Smooth step function: 3t^2 - 2t^3 (smooth S-curve)
    smooth_t = 3*t.^2 - 2*t.^3;
    jump_profile = seg.height * smooth_t;  % Positive for upward jump
    
    z_flat = zeros(size(x));
    z_flat(jump_indices) = jump_profile;
else
    z_flat = zeros(size(x));
end

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_drop_segment(seg, slope_start, resolution)
% Generate drop segment with sudden step down

if ~isfield(seg, 'height')
    error('Drop segment requires height field');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% First create flat segment with sudden drop (zero gradient)
drop_position = seg.distance * 0.5;  % Drop at middle of segment

% Create step drop profile
z_flat = zeros(size(x));
drop_indices = x >= drop_position;
z_flat(drop_indices) = -seg.height;  % Negative for downward step

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_staircase_segment(seg, slope_start, resolution)
% Generate staircase segment with downward steps

if ~isfield(seg, 'height') || ~isfield(seg, 'step_length') || ~isfield(seg, 'num_steps')
    error('Staircase segment requires height, step_length, and num_steps fields');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% Calculate staircase parameters
total_staircase_length = seg.step_length * seg.num_steps;
staircase_start = (seg.distance - total_staircase_length) / 2;  % Center the staircase
staircase_end = staircase_start + total_staircase_length;

% Calculate step height (total height divided by number of steps)
step_height = seg.height / seg.num_steps;

% First create flat segment with staircase (zero gradient)
z_flat = zeros(size(x));

% Create staircase profile
for i = 1:seg.num_steps
    step_start = staircase_start + (i-1) * seg.step_length;
    step_end = staircase_start + i * seg.step_length;
    
    % Find indices for this step
    step_indices = (x >= step_start) & (x <= step_end);
    
    if any(step_indices)
        % Each step is a horizontal platform at the step height
        z_flat(step_indices) = -i * step_height;
    end
end

% Lower the entire course after the staircase by the total staircase height
% This ensures the course continues at the lowered level
after_staircase_indices = x > staircase_end;
if any(after_staircase_indices)
    z_flat(after_staircase_indices) = -seg.height;
end

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_trapezium_segment(seg, slope_start, resolution)
% Generate trapezium segment with upramp, flat section, and downramp

if ~isfield(seg, 'height') || ~isfield(seg, 'flat_length')
    error('Trapezium segment requires height and flat_length fields');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% Calculate ramp parameters
if isfield(seg, 'ramp_length')
    % User specified ramp length
    ramp_length = seg.ramp_length;
elseif isfield(seg, 'ramp_inclination')
    % User specified ramp inclination (in degrees)
    ramp_inclination_rad = seg.ramp_inclination * pi / 180;
    ramp_length = seg.height / tan(ramp_inclination_rad);
else
    % Default: 20 degree inclination
    ramp_inclination_rad = 20 * pi / 180;
    ramp_length = seg.height / tan(ramp_inclination_rad);
end

% Calculate trapezium parameters
total_trapezium_length = 2 * ramp_length + seg.flat_length;
trapezium_start = (seg.distance - total_trapezium_length) / 2;  % Center the trapezium
trapezium_end = trapezium_start + total_trapezium_length;

% Define trapezium sections
upramp_start = trapezium_start;
upramp_end = upramp_start + ramp_length;
flat_start = upramp_end;
flat_end = flat_start + seg.flat_length;
downramp_start = flat_end;
downramp_end = downramp_start + ramp_length;

% First create flat segment with trapezium (zero gradient)
z_flat = zeros(size(x));

% Create upramp (smooth step function)
upramp_indices = (x >= upramp_start) & (x <= upramp_end);
if any(upramp_indices)
    upramp_x = x(upramp_indices) - upramp_start;
    t = upramp_x / ramp_length;  % Normalize to [0,1]
    
    % Smooth step function: 3t^2 - 2t^3 (smooth S-curve)
    smooth_t = 3*t.^2 - 2*t.^3;
    upramp_profile = seg.height * smooth_t;  % Positive for upward ramp
    
    z_flat(upramp_indices) = upramp_profile;
end

% Create flat section at full height
flat_indices = (x >= flat_start) & (x <= flat_end);
if any(flat_indices)
    z_flat(flat_indices) = seg.height;
end

% Create downramp (smooth step function, inverted)
downramp_indices = (x >= downramp_start) & (x <= downramp_end);
if any(downramp_indices)
    downramp_x = x(downramp_indices) - downramp_start;
    t = downramp_x / ramp_length;  % Normalize to [0,1]
    
    % Smooth step function: 3t^2 - 2t^3 (smooth S-curve)
    smooth_t = 3*t.^2 - 2*t.^3;
    downramp_profile = seg.height * (1 - smooth_t);  % Downward from full height
    
    z_flat(downramp_indices) = downramp_profile;
end

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_gap_jump_segment(seg, slope_start, resolution)
% Generate gap jump segment with upramp, gap, and downramp

if ~isfield(seg, 'height') || ~isfield(seg, 'gap_width')
    error('Gap jump segment requires height and gap_width fields');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% Calculate ramp parameters
if isfield(seg, 'ramp_length')
    % User specified ramp length
    ramp_length = seg.ramp_length;
elseif isfield(seg, 'ramp_inclination')
    % User specified ramp inclination (in degrees)
    ramp_inclination_rad = seg.ramp_inclination * pi / 180;
    ramp_length = seg.height / tan(ramp_inclination_rad);
else
    % Default: 10 degree inclination
    ramp_inclination_rad = 10 * pi / 180;
    ramp_length = seg.height / tan(ramp_inclination_rad);
end

% Calculate gap jump parameters
total_gap_jump_length = 2 * ramp_length + seg.gap_width;
gap_jump_start = (seg.distance - total_gap_jump_length) / 2;  % Center the gap jump
gap_jump_end = gap_jump_start + total_gap_jump_length;

% Define gap jump sections
upramp_start = gap_jump_start;
upramp_end = upramp_start + ramp_length;
gap_start = upramp_end;
gap_end = gap_start + seg.gap_width;
downramp_start = gap_end;
downramp_end = downramp_start + ramp_length;

% First create flat segment with gap jump (zero gradient)
z_flat = zeros(size(x));

% Create upramp (smooth step function)
upramp_indices = (x >= upramp_start) & (x <= upramp_end);
if any(upramp_indices)
    upramp_x = x(upramp_indices) - upramp_start;
    t = upramp_x / ramp_length;  % Normalize to [0,1]
    
    % Smooth step function: 3t^2 - 2t^3 (smooth S-curve)
    smooth_t = 3*t.^2 - 2*t.^3;
    upramp_profile = seg.height * smooth_t;  % Positive for upward ramp
    
    z_flat(upramp_indices) = upramp_profile;
end

% Create gap (no profile - just air)
gap_indices = (x >= gap_start) & (x <= gap_end);
if any(gap_indices)
    z_flat(gap_indices) = 0;  % Gap is at ground level
end

% Create downramp (smooth step function, inverted)
downramp_indices = (x >= downramp_start) & (x <= downramp_end);
if any(downramp_indices)
    downramp_x = x(downramp_indices) - downramp_start;
    t = downramp_x / ramp_length;  % Normalize to [0,1]
    
    % Smooth step function: 3t^2 - 2t^3 (smooth S-curve)
    smooth_t = 3*t.^2 - 2*t.^3;
    downramp_profile = seg.height * (1 - smooth_t);  % Downward from full height
    
    z_flat(downramp_indices) = downramp_profile;
end

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_rock_garden_segment(seg, slope_start, resolution)
% Generate rock garden segment with randomly placed rocks

if ~isfield(seg, 'height') || ~isfield(seg, 'rock_density')
    error('Rock garden segment requires height and rock_density fields');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% First create flat segment with rock garden (zero gradient)
z_flat = zeros(size(x));

% Generate random rock positions
num_rocks = round(seg.distance * seg.rock_density);
rock_positions = sort(rand(1, num_rocks) * seg.distance);

% Create rock profile
for i = 1:num_rocks
    rock_x = rock_positions(i);
    rock_height = seg.height * (0.3 + 0.7 * rand());  % Random height between 30% and 100% of max height
    rock_width = 0.2 + 0.3 * rand();  % Random width between 0.2 and 0.5m
    
    % Find indices for this rock
    rock_indices = (x >= rock_x - rock_width/2) & (x <= rock_x + rock_width/2);
    
    if any(rock_indices)
        % Create rock profile (smooth bump)
        rock_x_local = x(rock_indices) - rock_x;
        t = rock_x_local / (rock_width/2);  % Normalize to [-1,1]
        
        % Smooth rock profile using cosine function
        rock_profile = rock_height * max(0, cos(pi * t / 2));
        
        z_flat(rock_indices) = max(z_flat(rock_indices), rock_profile);
    end
end

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_root_section_segment(seg, slope_start, resolution)
% Generate root section segment with exposed tree roots

if ~isfield(seg, 'height') || ~isfield(seg, 'root_density')
    error('Root section segment requires height and root_density fields');
end

x = linspace(0, seg.distance, round(seg.distance * resolution) + 1);

% First create flat segment with root section (zero gradient)
z_flat = zeros(size(x));

% Generate random root positions
num_roots = round(seg.distance * seg.root_density);
root_positions = sort(rand(1, num_roots) * seg.distance);

% Create root profile
for i = 1:num_roots
    root_x = root_positions(i);
    root_height = seg.height * (0.2 + 0.8 * rand());  % Random height between 20% and 100% of max height
    root_width = 0.1 + 0.2 * rand();  % Random width between 0.1 and 0.3m
    root_angle = (rand() - 0.5) * pi/6;  % Random angle between -15 and +15 degrees
    
    % Find indices for this root
    root_indices = (x >= root_x - root_width/2) & (x <= root_x + root_width/2);
    
    if any(root_indices)
        % Create root profile (smooth bump with slight angle)
        root_x_local = x(root_indices) - root_x;
        t = root_x_local / (root_width/2);  % Normalize to [-1,1]
        
        % Smooth root profile using cosine function with angle
        root_profile = root_height * max(0, cos(pi * t / 2));
        
        % Add slight angle to the root
        root_profile = root_profile + root_x_local * tan(root_angle);
        
        z_flat(root_indices) = max(z_flat(root_indices), root_profile);
    end
end

% Then apply gradient by rotating
z = z_flat + seg.gradient * x;

% Final slope
slope_end = slope_start + seg.gradient;

end

function [x, z, slope_end] = generate_curve_segment(seg, slope_start, resolution)
% Generate curve segment with smooth transition between gradients using smooth_fillet

if ~isfield(seg, 'final_gradient') || ~isfield(seg, 'R_curve')
    error('Curve segment requires final_gradient and R_curve fields');
end

% Use the robust smooth_fillet function
% Split the segment into two equal parts for the two straight sections
d1 = seg.distance / 2;
d2 = seg.distance / 2;

% Generate the smooth curve
[x, z] = smooth_fillet(0, 0, d1, d2, seg.gradient, seg.final_gradient, seg.R_curve);

% Final slope is the final gradient
slope_end = seg.final_gradient;

end

% =========================================================================
%                            PLOTTING FUNCTION
% =========================================================================

function plot_segments(segments, resolution)
% Plot each segment individually for debugging

n_segments = length(segments);
n_cols = min(3, n_segments);
n_rows = ceil(n_segments / n_cols);

figure('Name', 'Individual Segment Analysis');
current_x = 0;
current_z = 0;
current_slope = 0;

for i = 1:n_segments
    subplot(n_rows, n_cols, i);
    
    seg = segments{i};
    
    % Generate segment
    [seg_x, seg_z, seg_slope_end] = generate_segment(seg, current_slope, resolution);
    
    % Offset segment to current position
    if i == 1
        seg_x = seg_x + current_x;
        seg_z = seg_z + current_z;
    else
        seg_x = seg_x - seg_x(1) + current_x;
        seg_z = seg_z - seg_z(1) + current_z;
    end
    
    % Plot the segment
    plot(seg_x, seg_z, 'b-', 'LineWidth', 2);
    hold on;
    
    % Mark start and end points
    plot(seg_x(1), seg_z(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(seg_x(end), seg_z(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Add feature markers for jumps and drops
    if strcmpi(seg.type, 'jump') && isfield(seg, 'height')
        jump_start = seg_x(1) + seg.distance * 0.3;
        jump_end = seg_x(1) + seg.distance * 0.7;
        jump_mid = (jump_start + jump_end) / 2;
        jump_z = interp1(seg_x, seg_z, jump_mid);
        plot(jump_mid, jump_z, '^', 'MarkerSize', 10, 'MarkerFaceColor', 'c');
        text(jump_mid, jump_z + 0.1, sprintf('Jump: %.1fm', seg.height), 'HorizontalAlignment', 'center');
    elseif strcmpi(seg.type, 'drop') && isfield(seg, 'height')
        drop_start = seg_x(1) + seg.distance * 0.4;
        drop_end = seg_x(1) + seg.distance * 0.6;
        drop_mid = (drop_start + drop_end) / 2;
        drop_z = interp1(seg_x, seg_z, drop_mid);
        plot(drop_mid, drop_z, 'v', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
        text(drop_mid, drop_z - 0.1, sprintf('Drop: %.1fm', seg.height), 'HorizontalAlignment', 'center');
    elseif strcmpi(seg.type, 'bumpy') && isfield(seg, 'height')
        text(seg_x(1), seg_z(1) + 0.1, sprintf('Bumps: %.1fm, %.1f/m', seg.height, seg.frequency), 'FontSize', 8);
    elseif strcmpi(seg.type, 'staircase') && isfield(seg, 'height')
        staircase_start = seg_x(1) + (seg.distance - seg.step_length * seg.num_steps) / 2;
        staircase_end = staircase_start + seg.step_length * seg.num_steps;
        staircase_mid = (staircase_start + staircase_end) / 2;
        staircase_z = interp1(seg_x, seg_z, staircase_mid);
        plot(staircase_mid, staircase_z, 's', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
        text(staircase_mid, staircase_z + 0.1, sprintf('Stairs: %.1fm, %d steps', seg.height, seg.num_steps), 'HorizontalAlignment', 'center');
    elseif strcmpi(seg.type, 'trapezium') && isfield(seg, 'height')
        % Calculate trapezium position
        if isfield(seg, 'ramp_length')
            ramp_length = seg.ramp_length;
        elseif isfield(seg, 'ramp_inclination')
            ramp_inclination_rad = seg.ramp_inclination * pi / 180;
            ramp_length = seg.height / tan(ramp_inclination_rad);
        else
            ramp_inclination_rad = 20 * pi / 180;
            ramp_length = seg.height / tan(ramp_inclination_rad);
        end
        total_trapezium_length = 2 * ramp_length + seg.flat_length;
        trapezium_start = seg_x(1) + (seg.distance - total_trapezium_length) / 2;
        trapezium_end = trapezium_start + total_trapezium_length;
        trapezium_mid = (trapezium_start + trapezium_end) / 2;
        trapezium_z = interp1(seg_x, seg_z, trapezium_mid);
        plot(trapezium_mid, trapezium_z, 'd', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
        text(trapezium_mid, trapezium_z + 0.1, sprintf('Trapezium: %.1fm, %.1fm flat', seg.height, seg.flat_length), 'HorizontalAlignment', 'center');
    elseif strcmpi(seg.type, 'gap_jump') && isfield(seg, 'height')
        % Calculate gap jump position
        if isfield(seg, 'ramp_length')
            ramp_length = seg.ramp_length;
        elseif isfield(seg, 'ramp_inclination')
            ramp_inclination_rad = seg.ramp_inclination * pi / 180;
            ramp_length = seg.height / tan(ramp_inclination_rad);
        else
            ramp_inclination_rad = 25 * pi / 180;
            ramp_length = seg.height / tan(ramp_inclination_rad);
        end
        total_gap_jump_length = 2 * ramp_length + seg.gap_width;
        gap_jump_start = seg_x(1) + (seg.distance - total_gap_jump_length) / 2;
        gap_jump_end = gap_jump_start + total_gap_jump_length;
        gap_jump_mid = (gap_jump_start + gap_jump_end) / 2;
        gap_jump_z = interp1(seg_x, seg_z, gap_jump_mid);
        plot(gap_jump_mid, gap_jump_z, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        text(gap_jump_mid, gap_jump_z + 0.1, sprintf('Gap Jump: %.1fm, %.1fm gap', seg.height, seg.gap_width), 'HorizontalAlignment', 'center');
    elseif strcmpi(seg.type, 'rock_garden') && isfield(seg, 'height')
        text(seg_x(1), seg_z(1) + 0.1, sprintf('Rock Garden: %.1fm, %.1f/m', seg.height, seg.rock_density), 'FontSize', 8);
    elseif strcmpi(seg.type, 'root_section') && isfield(seg, 'height')
        text(seg_x(1), seg_z(1) + 0.1, sprintf('Root Section: %.1fm, %.1f/m', seg.height, seg.root_density), 'FontSize', 8);
    elseif strcmpi(seg.type, 'curve') && isfield(seg, 'R_curve')
        curve_mid = seg_x(1) + seg.distance / 2;
        curve_z = interp1(seg_x, seg_z, curve_mid);
        plot(curve_mid, curve_z, 'h', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        text(curve_mid, curve_z + 0.1, sprintf('Curve: R=%.1fm, %.2f→%.2f', seg.R_curve, seg.gradient, seg.final_gradient), 'HorizontalAlignment', 'center');
    end
    
    % Formatting
    title(sprintf('Segment %d: %s (%.1fm, grad=%.2f)', i, seg.type, seg.distance, seg.gradient));
    xlabel('Distance (m)');
    ylabel('Elevation (m)');
    grid on;
    axis equal;
    
    % Add slope information
    slope_text = sprintf('Start slope: %.3f\nEnd slope: %.3f', current_slope, seg_slope_end);
    text(0.02, 0.98, slope_text, 'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 8);
    
    % Update position for next segment
    current_x = seg_x(end);
    current_z = seg_z(end);
    current_slope = seg_slope_end;
end

sgtitle('Individual Segment Analysis - Green=Start, Red=End, Cyan=Jump, Magenta=Drop, Yellow=Staircase, Black=Trapezium, Red=Gap Jump, Green=Curve');

end

% =========================================================================
%                        SMOOTH FILLET FUNCTION
% =========================================================================

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
% - The second straight starts at the "knee" point and goes in direction grad2
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