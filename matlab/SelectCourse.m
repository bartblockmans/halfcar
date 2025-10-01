function [x_course, z_course, course_name, course_data] = SelectCourse(varargin)
%SELECTCOURSE Interactive course selection and visualization
%
% SYNTAX:
%   [course_name, course_data, x_course, z_course] = SelectCourse()
%   [course_name, course_data, x_course, z_course] = SelectCourse('course', course_name)
%   [course_name, course_data, x_course, z_course] = SelectCourse('resolution', res)
%   [course_name, course_data, x_course, z_course] = SelectCourse('preview', true)
%
% INPUTS:
%   'course'     - Course name to select directly (string), default = interactive selection
%   'resolution' - Point density (points per meter), default = 10
%   'preview'    - Show course preview, default = true
%
% OUTPUTS:
%   course_name  - Selected course name (string)
%   course_data  - Course definition struct
%   x_course     - X coordinates of course (m)
%   z_course     - Z coordinates of course (m)
%
% EXAMPLE:
%   [name, data, x, z] = SelectCourse();
%   [name, data, x, z] = SelectCourse('course', 'complex_downhill');
%   [name, data, x, z] = SelectCourse('resolution', 20, 'preview', false);

% Parse inputs
p = inputParser;
addParameter(p, 'course', '', @ischar);
addParameter(p, 'resolution', 10, @(x) isnumeric(x) && x > 0);
addParameter(p, 'preview', true, @islogical);
parse(p, varargin{:});

course_name_input = p.Results.course;
resolution = p.Results.resolution;
show_preview = p.Results.preview;

% Load course library
courses = CourseLibrary();

% Get course names
course_names = fieldnames(courses);
n_courses = length(course_names);

if n_courses == 0
    error('No courses found in course library');
end

% Check if course name was provided directly
if ~isempty(course_name_input)
    % Direct course selection
    if isfield(courses, course_name_input)
        course_name = course_name_input;
        course_data = courses.(course_name);
        fprintf('Selected course: %s\n', course_data.name);
    else
        error('Course "%s" not found. Available courses: %s', course_name_input, strjoin(course_names, ', '));
    end
else
    % Interactive course selection
    fprintf('\n=== AVAILABLE COURSES ===\n');
    for i = 1:n_courses
        course = courses.(course_names{i});
        fprintf('%2d. %s\n', i, course.name);
        fprintf('    %s\n', course.description);
        
        % Check if segments field exists and get length safely
        if isfield(course, 'segments') && iscell(course.segments)
            n_segments = length(course.segments);
        else
            n_segments = 0;
        end
        fprintf('    Segments: %d\n', n_segments);
        fprintf('\n');
    end

    % Get user selection
    while true
        try
            selection = input(sprintf('Select course (1-%d): ', n_courses));
            if isnumeric(selection) && selection >= 1 && selection <= n_courses
                break;
            else
                fprintf('Invalid selection. Please enter a number between 1 and %d.\n', n_courses);
            end
        catch
            fprintf('Invalid input. Please enter a number.\n');
        end
    end

    % Get selected course
    course_name = course_names{selection};
    course_data = courses.(course_name);
end

fprintf('\nSelected: %s\n', course_data.name);
fprintf('Description: %s\n', course_data.description);

% Generate course
fprintf('Generating course...\n');
[x_course, z_course, course_info] = ConstructCourse(course_data.segments, 'resolution', resolution);

% Add course_info fields to course_data for easy access
course_data.total_distance = course_info.total_distance;
course_data.segment_boundaries = course_info.segment_boundaries;

% Display course statistics
fprintf('Course generated successfully!\n');
fprintf('Total distance: %.1f m\n', course_info.total_distance);
fprintf('Number of points: %d\n', length(x_course));
fprintf('Resolution: %.1f points/m\n', resolution);

% Show course preview if requested
if show_preview
    show_course_preview(x_course, z_course, course_data, course_info);
end

end

% =========================================================================
%                            PREVIEW FUNCTION
% =========================================================================

function show_course_preview(x_course, z_course, course_data, course_info)
% Show course preview with segment information

figure('Name', sprintf('Course Preview: %s', course_data.name), ...
       'Position', [100, 100, 1200, 600]);

% Main course plot
subplot(2,1,1);
plot(x_course, z_course, 'b-', 'LineWidth', 2);
hold on;

% Mark segment boundaries
segment_boundaries = course_info.segment_boundaries;
for i = 1:length(segment_boundaries)
    x_boundary = segment_boundaries(i);
    z_boundary = interp1(x_course, z_course, x_boundary);
    plot(x_boundary, z_boundary, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end

% Mark start and end
plot(x_course(1), z_course(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(x_course(end), z_course(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Add segment labels in the middle above each segment
n_segments = length(course_data.segments);
for i = 1:n_segments
    if i == 1
        % First segment: from start to first boundary
        x_start = x_course(1);
        x_end = segment_boundaries(1);
    else
        % Other segments: from previous boundary to current boundary
        x_start = segment_boundaries(i-1);
        x_end = segment_boundaries(i);
    end
    
    % Calculate middle position
    x_middle = (x_start + x_end) / 2;
    z_middle = interp1(x_course, z_course, x_middle);
    
    % Add label above the segment
    text(x_middle, z_middle + 2.0, sprintf('S%d', i), ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 12);
end

% Add start and end labels
text(x_course(1), z_course(1) + 0.2, 'START', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(x_course(end), z_course(end) + 0.2, 'END', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

xlabel('Distance (m)');
ylabel('Elevation (m)');
title(sprintf('%s - Course Overview', course_data.name));
grid on;
axis equal;

% Segment details table
subplot(2,1,2);
axis off;

% Create segment information table
n_segments = length(course_data.segments);
table_data = cell(n_segments + 1, 5);
table_data(1,:) = {'Segment', 'Type', 'Distance (m)', 'Gradient', 'Features'};

for i = 1:n_segments
    seg = course_data.segments{i};
    table_data{i+1,1} = sprintf('%d', i);
    table_data{i+1,2} = seg.type;
    table_data{i+1,3} = sprintf('%.1f', seg.distance);
    table_data{i+1,4} = sprintf('%.3f', seg.gradient);
    
    % Features column
    features = '';
    if strcmpi(seg.type, 'bumpy') && isfield(seg, 'height')
        features = sprintf('Height: %.2fm, Freq: %.1f/m', seg.height, seg.frequency);
    elseif strcmpi(seg.type, 'jump') && isfield(seg, 'height')
        features = sprintf('Height: %.2fm', seg.height);
    elseif strcmpi(seg.type, 'drop') && isfield(seg, 'height')
        features = sprintf('Height: %.2fm', seg.height);
    end
    table_data{i+1,5} = features;
end

% Calculate optimal table width based on content
col_widths = {60, 80, 100, 80, 250}; % Increased last column width
total_width = sum([col_widths{:}]) + 20; % Add some padding

% Center the table and position it at the bottom
table_x = (1200 - total_width) / 2; % Center horizontally
table_y = 20; % Position at bottom
table_height = 150; % Reduced height
table_handle = uitable('Data', table_data(2:end,:), ...
                      'ColumnName', table_data(1,:), ...
                      'Position', [table_x, table_y, total_width, table_height], ...
                      'ColumnWidth', col_widths, ...
                      'ColumnEditable', false, ...
                      'Enable', 'inactive');

% Add course statistics
stats_text = sprintf('Total Distance: %.1f m | Total Segments: %d | Points: %d', ...
                    course_info.total_distance, n_segments, length(x_course));
text(0.5, 0.95, stats_text, 'Units', 'normalized', ...
     'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');

end