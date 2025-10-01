%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Project:     2D Springs Plot
%
% Filename:    Spring.m
%
% Author:      Xue-She Wang
%
% E-mail:      xueshe.wang@duke.edu 
%              xueshe.wang@gmail.com
%
% Description: This class file 'Spring' generates coordinates of a 2D
%              spring for plotting. A new spring instance can be created by
%              passing 'spring radius' and 'number of coils'. E.g. 
%              'spr = Spring(5, 10);' creates a instance a spring with 
%              radius of 5 and coils number of 10. The class function 
%              'getSpr' is used to give the coordinates of the spring
%              connecting two arbitrary points. E.g. '[x, y] = spr.getSpr(
%              [1, 2], [4, 3]);' returns the data of the spring connecting
%              point (1, 2) and point (4, 3), and then you could simply use
%              'plot(x, y)' to visualize the spring. More details can be
%              found in the demo files.
%
% Revision:    2018/11/20
%              - first release.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Spring
    
    properties (Access = private)
        spr0_x, spr0_y, spr0_len
    end
    
    %% Public Methods
    methods        
        % Constructor
        function obj = Spring(R, num_coils)
            [obj.spr0_x, obj.spr0_y, obj.spr0_len] = obj.sprInit(R, num_coils);
        end
        
        % Get coordinates of transformed spring
        function [x, y] = getSpr(obj, pt1, pt2)
            % Scale
            scale = sqrt((pt1(1) - pt2(1)) ^ 2 + (pt1(2) - pt2(2)) ^ 2) / obj.spr0_len;
            x = obj.spr0_x;
            y = obj.spr0_y * scale;
            [theta, r] = cart2pol(x, y);    

            % Rotate
            alpha = pi / 2 - atan((pt2(2) - pt1(2)) / (pt2(1) - pt1(1)));
            if pt1(1) > pt2(1)
                alpha = alpha + pi;
            end
            theta = theta - alpha;
            [x, y] = pol2cart(theta, r);

            % Move
            x = x + pt1(1);
            y = y + pt1(2);            
        end
    end
    
    %% Private Methods
    methods (Access = private)
        % Create initial spring
        function [x, y, len] = sprInit(~, R, n)

            r = 0.7 * R;
            step = 2 * (R - r);

            ang_R = linspace(3 * pi / 2, pi / 2, 20);
            ang_r = linspace(pi / 2, -pi / 2, 20);

            x = [];
            y = [];

            c_R = 0;

            for i = 1 : n

                c_R = c_R + step;
                R_x = R * cos(ang_R);
                R_y = c_R + R * sin(ang_R);

                c_r = c_R + R - r;
                r_x = R * cos(ang_r);
                r_y = c_r + r * sin(ang_r);

                x = [x, R_x, r_x];
                y = [y, R_y, r_y];

                if i == n
                    c_R = c_R + step;
                    R_x = R * cos(ang_R);
                    R_y = c_R + R * sin(ang_R);

                    % Add straight lines at the spring's two ends
                    x = [0, x, R_x, 0]; 
                    y = [y(1) - R, y, R_y, y(end) + 3 * R];  

                    % Move the spring's start point to origin
                    y = y - min(y);
                    break;
                end

            end

            len = abs(y(end) - y(1));

        end
    end
    
    
end