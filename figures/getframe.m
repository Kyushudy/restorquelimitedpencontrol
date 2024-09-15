% Step 1: Create a VideoReader object
video = VideoReader('video.mp4'); % Replace 'your_video.mp4' with the actual video filename

% Step 2: Extract frames from the video
frame_number = 1; % Initialize frame counter

while hasFrame(video)
    % Read the next frame
    frame = readFrame(video);
    
    % Optional: Display the frame
    % imshow(frame);
    
    % Save frame as an image (optional)
    filename = sprintf('frames/frame_%04d.png', frame_number); % Save frame as .png with 4-digit number
    imwrite(frame, filename);
    
    % Increment frame counter
    frame_number = frame_number + 1;
    
    % Pause for visualizing frames (optional)
    % pause(0.1); % Adjust pause as necessary
end