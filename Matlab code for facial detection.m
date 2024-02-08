%%%%%%%%%%%%%%%%%%%%%%%%%%%% MATLAB CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Disable the warning related to old coordinate system in the vision library
warning('off','vision:transition:usesOldCoordinates')

% Clear all variables in the workspace
clear all

% Clear the command window
clc

% Set the answer variable to 1
answer=1;

% Create a serial object to communicate with Arduino on COM6 port with baud rate 9600
arduino=serial('COM6','BaudRate',9600);
fopen(arduino);

% Create a face detector object using the Viola-Jones algorithm
faceDetector = vision.CascadeObjectDetector();

% Create a video device object to acquire images from the webcam
obj =imaq.VideoDevice('winvideo', 1, 'I420_320x240','ROI', [1 1 320 240]);

% Set the color space of the acquired images to RGB
set(obj,'ReturnedColorSpace', 'rgb');

% Create a figure to display the webcam feed
figure('menubar','none','tag','webcam');

% Initialize the wait counter
wait=0;

% Loop until the wait counter reaches 600
while (wait<600)
    wait=wait+1;
    
    % Acquire a frame from the video device
    frame=step(obj);
    
    % Detect faces in the frame using the face detector
    bbox=step(faceDetector,frame);
    
    % Print the current value of the wait counter
    wait
    
    % Check if any faces are detected
    if(~isempty(bbox))
        % Print the bounding box coordinates of the detected face
        bbox
        
        % Calculate the center coordinates of the face
        centx=bbox(1) + (bbox(3)/2) ;
        centy=bbox(2) - (bbox(4)/2) ;
        
        % Print the x and y coordinates of the center
        c1=(centx);
        c2=(centy);
        c1
        c2
        
        % Send the center coordinates to Arduino
        fprintf(arduino,'%s',char(centx));      
        fprintf(arduino,'%s',char(centy));
    end
    
    % Create a shape inserter object to draw bounding boxes
    boxInserter  = vision.ShapeInserter('BorderColor','Custom',...
    'CustomBorderColor',[255 0 255]);
    
    % Insert bounding boxes around the detected faces
    videoOut = step(boxInserter, frame,bbox);
    
    % Display the processed frame with bounding boxes
    imshow(videoOut,'border','tight');
    
    % Check if the webcam figure is closed
    f=findobj('tag','webcam');
    
    % If the figure is closed, perform additional processing
    if (isempty(f));
        % Convert the frame to the hue channel
        [hueChannel,~,~] = rgb2hsv(frame);
        
        % Display the hue channel data and draw the bounding box around the face
        rectangle('Position',bbox(1,:),'LineWidth',2,'EdgeColor',[1 1 0])
        
        % Clear the current figure
        hold off
        
        % Create a nose detector object
        noseDetector = vision.CascadeObjectDetector('Nose');
        
        % Crop the face image using the bounding box
        faceImage    = imcrop(frame,bbox);
        
        % Detect the nose in the face image
        noseBBox     = step(noseDetector,faceImage);
        
        % Adjust the nose bounding box coordinates
        noseBBox(1:1) = noseBBox(1:1) + bbox(1:1);
        
        % Get the video information
        videoInfo    = info(obj);
        
        % Get the region of interest (ROI) from the video device
        ROI=get(obj,'ROI');
        
        % Set the video size based on the ROI
        VideoSize = [ROI(3) ROI(4)];
        
        % Create a video player object to display the processed video
        videoPlayer  = vision.VideoPlayer('Position',[300 300 VideoSize+60]);
        
        % Create a histogram-based tracker object
        tracker = vision.HistogramBasedTracker;
        
        % Initialize the tracker with the hue channel and the face bounding box
        initializeObject(tracker, hueChannel, bbox);
        
        % Initialize the time counter
        time=0;
        
        % Loop until the time counter reaches 600
        while (time<600)
            time=time+1;
            
            % Extract the next video frame
    frame = step(obj);
    time
% RGB -> HSV
    [hueChannel,~,~] = rgb2hsv(frame);
    % Track using the Hue channel data
    bbox = step(tracker, hueChannel);
    % Insert a bounding box around the object being tracked
    videoOut = step(boxInserter, frame, bbox);
    %Insert text coordinates
    % Display the annotated video frame using the video player object
    step(videoPlayer, videoOut);
    pause (.2)
end
time
% Release resources
release(obj);
release(videoPlayer);
%release(vidobj);
        close(gcf)
        break
    end
    pause(0.05)
end
fclose(arduino);
%release(vidobj);
release(obj);
%release(videoPlayer);