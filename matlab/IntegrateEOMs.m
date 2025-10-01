function [t, y] = IntegrateEOMs(y0, P, road, Tend, solver, debug)

% Check input arguments
if nargin < 5; solver = 'ode15s'; end
if nargin < 6; debug = 0; end

% Initialize progress and storage settings
report_interval = 100;  % Report progress and save results every N steps

% Set solver options
opts = odeset('RelTol',1e-3,'AbsTol',1e-6,'MaxStep',0.005,...
              'OutputFcn',@(t,y,flag) progress_output(t,y,flag,Tend,report_interval,road));

% Define right-hand side
rhs  = @(t,y) rhs5dof(t, y, P, road);

% Check if course-based simulation (Tend = Inf)
if isinf(Tend)
    % Add course end event detection
    course_length = road.x_course(end) - road.x_course(1);
    opts = odeset(opts, 'Events', @(t,y) course_end_event(t, y, course_length));
    Tend = 1e6; % Large time span (will stop at event)
end

% t span
t_span = [0 Tend];

% Switch solver
switch solver
    case 'ode15s'
        fprintf('Integrating with ode15s (stiff problems, variable step) ...\n'); tic;
        [t, y] = ode15s(rhs, t_span, y0, opts);
        
    case 'ode45'
        fprintf('Integrating with ode45 (non-stiff problems, variable step) ...\n'); tic;
        [t, y] = ode45(rhs, t_span, y0, opts);
        
    case 'ode23s'
        fprintf('Integrating with ode23s (stiff problems, low accuracy) ...\n'); tic;
        [t, y] = ode23s(rhs, t_span, y0, opts);
        
    case 'ode23t'
        fprintf('Integrating with ode23t (moderately stiff, trapezoidal) ...\n'); tic;
        [t, y] = ode23t(rhs, t_span, y0, opts);
        
    case 'ode23tb'
        fprintf('Integrating with ode23tb (stiff problems, TR-BDF2) ...\n'); tic;
        [t, y] = ode23tb(rhs, t_span, y0, opts);
        
    otherwise
        error('Unknown solver: %s. Available solvers: ode15s, ode45, ode23s, ode23t, ode23tb', solver);
end

fprintf('Done in %.2f s.\n', toc);

% =========================================================================
%                            RIGHT-HAND SIDE
% =========================================================================

function dydt = rhs5dof(t, y, P, road)

    dydt = ComputeAccelerations(t, y, P, road, debug);

end

% Progress output function
function status = progress_output(t, y, flag, Tend, report_interval, road)
    persistent last_time step_count;
    
    if isempty(flag)
        % Called during integration
        if isempty(last_time)
            last_time = 0;
            step_count = 0;
            course_length = road.x_course(end) - road.x_course(1);
        end
        
        step_count = step_count + 1;
        current_time = t(end);
        dt = current_time - last_time;
        
        % Calculate progress based on simulation type
        if isinf(Tend)
            % Course-based progress
            current_x = y(1); % X position
            progress = (current_x / course_length) * 100;
        else
            % Time-based progress
            progress = (current_time / Tend) * 100;
        end
        
        % Store intermediate results at specified interval
        if mod(step_count, report_interval) == 0
            % Create current result
            current_result = struct('time', current_time, 'y', y);
            
            % Append to existing results or create new
            if evalin('base', 'exist(''intermediate_results'', ''var'')')
                existing = evalin('base', 'intermediate_results');
                intermediate_results.time = [existing.time, current_time];
                intermediate_results.y = [existing.y; y'];
            else
                intermediate_results.time = current_time;
                intermediate_results.y = y';
            end
            
            % Save to workspace
            assignin('base', 'intermediate_results', intermediate_results);
        end
        
        % Print progress at specified interval or every 0.1 seconds
        if (current_time - last_time >= 0.1) || (mod(step_count, report_interval) == 0)
            if isinf(Tend)
                % Course-based progress
                fprintf('Progress: %.1f%% | Distance: %.1f/%.1f m | Time: %.3f s | dt: %.6f s | Step: %d\n', ...
                        progress, current_x, course_length, current_time, dt, step_count);
            else
                % Time-based progress
                fprintf('Progress: %.1f%% | Time: %.3f/%.3f s | dt: %.6f s | Step: %d\n', ...
                        progress, current_time, Tend, dt, step_count);
            end
            last_time = current_time;
        end
        
    elseif strcmp(flag, 'init')
        % Called at start of integration
        fprintf('Starting integration...\n');
        last_time = 0;
        step_count = 0;
        % Clear any existing intermediate results
        evalin('base', 'clear intermediate_results');
        
    elseif strcmp(flag, 'done')
        % Called at end of integration
        fprintf('Integration completed. Total steps: %d\n', step_count);
        % Clear intermediate results from workspace
        evalin('base', 'clear intermediate_results');
    end
    
    status = 0;  % Continue integration
end

% Event function for course end detection
function [value, isterminal, direction] = course_end_event(t, y, course_length)
    % Get current X position
    X = y(1);
    
    % Check if bicycle has reached the end of the course
    value = X - course_length;  % Zero when X = course_length
    isterminal = 1;             % Stop integration when event occurs
    direction = 1;              % Only detect when crossing from negative to positive
end

end