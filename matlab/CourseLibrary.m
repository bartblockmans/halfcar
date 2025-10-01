function courses = CourseLibrary()
%COURSELIBRARY Define all available courses for the downhill simulation
%
% This function returns a struct containing all predefined courses.
% Each course has a name, description, and segment definition.
%
% USAGE:
%   courses = CourseLibrary();
%   course = courses('my_course_name');
%
% COURSE DEFINITION:
%   Each course is defined by a cell array of segments with fields:
%   - type: 'flat', 'bumpy', 'jump', 'drop'
%   - distance: Length of segment (m)
%   - gradient: Road gradient (positive = uphill, negative = downhill)
%   - height: Feature height (m) - for bumpy/jump/drop segments
%   - frequency: Bumps per meter - for bumpy segments only

% Initialize course library
courses = struct();

% =========================================================================
%                            COURSE DEFINITIONS
% =========================================================================

% Basic test course
courses.basic_test.name = 'Basic Test Course';
courses.basic_test.description = 'Simple course with flat sections and one jump';
courses.basic_test.segments = {
    struct('type', 'flat', 'distance', 20, 'gradient', 0);
    struct('type', 'jump', 'distance', 10, 'gradient', -0.05, 'height', 0.5);
    struct('type', 'flat', 'distance', 30, 'gradient', 0);
};

% Complex downhill course
courses.complex_downhill.name = 'Complex Downhill';
courses.complex_downhill.description = 'Challenging course with bumps, jumps, and drops';
courses.complex_downhill.segments = {
    struct('type', 'flat', 'distance', 30, 'gradient', 0);
    struct('type', 'bumpy', 'distance', 20, 'gradient', -0.05, 'height', 0.03, 'frequency', 3);
    struct('type', 'jump', 'distance', 15, 'gradient', -0.1, 'height', 1.5);
    struct('type', 'bumpy', 'distance', 25, 'gradient', -0.15, 'height', 0.08, 'frequency', 1.5);
    struct('type', 'drop', 'distance', 12, 'gradient', -0.3, 'height', 1.2);
    struct('type', 'jump', 'distance', 18, 'gradient', -0.08, 'height', 0.8);
    struct('type', 'flat', 'distance', 40, 'gradient', 0);
};

% Extreme course
courses.extreme.name = 'Extreme Course';
courses.extreme.description = 'High-speed course with large features';
courses.extreme.segments = {
    struct('type', 'flat', 'distance', 10, 'gradient', -0.1);
    struct('type', 'jump', 'distance', 20, 'gradient', -0.2, 'height', 0.5);
    struct('type', 'drop', 'distance', 15, 'gradient', -0.4, 'height', 0.5);
    struct('type', 'bumpy', 'distance', 30, 'gradient', -0.15, 'height', 0.10, 'frequency', 2);
    struct('type', 'jump', 'distance', 40, 'gradient', -0.25, 'height', 0.5);
    struct('type', 'staircase', 'distance', 50, 'gradient', 0, 'height', 4.0, 'step_length', 0.5, 'num_steps', 16);
    struct('type', 'jump', 'distance', 30, 'gradient', -0.2, 'height', 1.0);
    struct('type', 'flat', 'distance', 20, 'gradient', 0);
    struct('type', 'trapezium', 'distance', 20, 'gradient', -0.1, 'height', 1.2, 'flat_length', 10.0, 'ramp_inclination', 15);
    struct('type', 'flat', 'distance', 20, 'gradient', 0);
    struct('type', 'rock_garden', 'distance', 30, 'gradient', -0.15, 'height', 0.2, 'rock_density', 5);
    struct('type', 'gap_jump', 'distance', 15, 'gradient', -0.1, 'height', 1.0, 'gap_width', 8.0);
    struct('type', 'root_section', 'distance', 30, 'gradient', -0.1, 'height', 0.1, 'root_density', 8);
    struct('type', 'flat', 'distance', 20, 'gradient', 0);
};

% Technical course
courses.technical.name = 'Technical Course';
courses.technical.description = 'Course with many small features requiring precision';
courses.technical.segments = {
    struct('type', 'flat', 'distance', 15, 'gradient', 0);
    struct('type', 'bumpy', 'distance', 10, 'gradient', -0.02, 'height', 0.02, 'frequency', 5);
    struct('type', 'jump', 'distance', 8, 'gradient', -0.03, 'height', 0.3);
    struct('type', 'bumpy', 'distance', 12, 'gradient', -0.05, 'height', 0.04, 'frequency', 4);
    struct('type', 'drop', 'distance', 6, 'gradient', -0.08, 'height', 0.4);
    struct('type', 'bumpy', 'distance', 15, 'gradient', -0.06, 'height', 0.03, 'frequency', 6);
    struct('type', 'jump', 'distance', 10, 'gradient', -0.04, 'height', 0.5);
    struct('type', 'flat', 'distance', 25, 'gradient', 0);
};

% Speed course
courses.speed.name = 'Speed Course';
courses.speed.description = 'Long downhill course optimized for high speed';
courses.speed.segments = {
    struct('type', 'flat', 'distance', 100, 'gradient', -0.05);
    struct('type', 'jump', 'distance', 30, 'gradient', -0.08, 'height', 1.0);
    struct('type', 'flat', 'distance', 80, 'gradient', -0.12);
    struct('type', 'jump', 'distance', 25, 'gradient', -0.15, 'height', 1.5);
    struct('type', 'flat', 'distance', 120, 'gradient', -0.1);
    struct('type', 'drop', 'distance', 20, 'gradient', -0.2, 'height', 1.8);
    struct('type', 'flat', 'distance', 150, 'gradient', 0);
};

% Beginner course
courses.beginner.name = 'Beginner Course';
courses.beginner.description = 'Easy course with gentle features';
courses.beginner.segments = {
    struct('type', 'flat', 'distance', 40, 'gradient', 0);
    struct('type', 'bumpy', 'distance', 20, 'gradient', -0.02, 'height', 0.01, 'frequency', 2);
    struct('type', 'jump', 'distance', 12, 'gradient', -0.03, 'height', 0.2);
    struct('type', 'flat', 'distance', 30, 'gradient', -0.01);
    struct('type', 'drop', 'distance', 8, 'gradient', -0.05, 'height', 0.3);
    struct('type', 'flat', 'distance', 50, 'gradient', 0);
};

% Add more courses here as needed...

end