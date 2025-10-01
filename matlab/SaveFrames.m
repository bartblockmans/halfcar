function SaveFrames(frames, folderPath, delayTime, create_gif, file_name)
    % Save frames variable in a .mat file, create an animated GIF, and an MP4 movie.
    %
    % Inputs:
    %   - frames     : Cell array containing frames (captured using getframe)
    %   - folderPath : Directory where to store the files

    if nargin < 3; delayTime = 0.2; end
    if nargin < 4; create_gif = 0; end

    % Ensure folder exists
    if ~isfolder(folderPath)
        mkdir(folderPath);
    end

    %% 1. Save the frames in a .mat file with the variable name as in Matlab
    matFilePath = fullfile(folderPath, [file_name, '.mat']);
    S.(file_name) = frames;  % Create struct with dynamic field name
    if length(frames) > 200
        save(matFilePath, '-struct', 'S','-v7.3'); % Save using the structure
    else
        save(matFilePath, '-struct', 'S'); % Save using the structure
    end

    %% 2. Create an animated GIF
    if create_gif
        gifFilePath = fullfile(folderPath, [file_name, '.gif']);
        
        % Loop through frames to write to GIF
        for i = 1:length(frames)
            % Convert frame to an image
            [imind, cm] = rgb2ind(frame2im(frames{i}), 256);
            
            % Write to the GIF file
            if i == 1
                imwrite(imind, cm, gifFilePath, 'gif', 'LoopCount', Inf, 'DelayTime', delayTime);
            else
                imwrite(imind, cm, gifFilePath, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
            end
        end
    end

    %% 3. Create an MP4 movie
    mp4FilePath = fullfile(folderPath, [file_name, '.mp4']);
    v = VideoWriter(mp4FilePath, 'MPEG-4');
    v.FrameRate = 1 / delayTime; % Set frame rate based on delay time
    open(v);

    % Write each frame to the MP4 file
    for i = 1:length(frames)
        writeVideo(v, frames{i}.cdata);
    end

    close(v); % Close the video file
end
