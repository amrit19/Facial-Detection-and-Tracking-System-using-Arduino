%%%%%%%%%%%%%%%%%%%%%%%%%%%% MATLAB CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
warning('off','vision:transition:usesOldCoordinates')
%warning control statement that enables u to indicate how u want Matlab 
%to act on certain warnings
clear all
clc
answer=1;
arduino=serial('COM6','BaudRate',9600);
fopen(arduino);
faceDetector = vision.CascadeObjectDetector();
%Detects Objects using the Viola-Jones algorithms
%CASCADEObject means passing object on to a succession of other object
%vision.CascadeObjectDetector() is a funtion or a package to detect faces
obj =imaq.VideoDevice('winvideo', 1, 'I420_320x240','ROI', [1 1 320 240]);
%imaq.VideoDevice it allows MATLAB to use Video Device of the system
%It also acquires images from the Image Acquisition Device 
%YUY2 is the format of the Camera supported by MATLAB
%ROI is Region Of Interest
%Get the input device using image acquisition toolbox,resolution = 640x480 to improve performance
set(obj,'ReturnedColorSpace', 'rgb');
%form is set(obj,name,value);
%sets the named property to the specified value for the Object obj.
%ReturnedColorSpace is a property that specifies the Color Space we want to
%the Toolbox to use when image data returns to Matlab WorkSpace 
figure('menubar','none','tag','webcam');
wait=0;
while (wait<600)
    wait=wait+1;
    frame=step(obj);
    %STEP Acquires a single frame from image acquisition Device
    %frame is the Variable assined to an image which is either RGB or
    %GRAYSCALE
    %Acquires a single frame from the VideoDevice System Object,obj.
    bbox=step(faceDetector,frame);
    wait
    if(~isempty(bbox))
        bbox
        centx=bbox(1) + (bbox(3)/2) ;
        centy=bbox(2) - (bbox(4)/2) ;
        c1=(centx);
        c2=(centy);
        c1
        c2
        fprintf(arduino,'%s',char(centx));      
        fprintf(arduino,'%s',char(centy));
    end
    %BBOX=Bounding Box
    %step returns a Matrix of M-by-4 where M is some Variable to bbox
    %M defines bounding boxes containing the detected objects
    %Each row in Matrix has 4 element Vector [x y width height] in pixels
    %The objects are detected from Image Named as 'frame'
    %detected objects are from face
    boxInserter  = vision.ShapeInserter('BorderColor','Custom',...
    'CustomBorderColor',[255 0 255]);
    %It inserts shapes according to matrix dimensions 
    %BorderColor is to specify the color of Shape by Default is Black
    %Here We set it to 'Custom' so we can use 'CustomBorderColor' to specify
    %the color of the border by vector representation
videoOut = step(boxInserter, frame,bbox);
%The Step function here returns an image
%Image consists of a Bounding box for the frame
%The BoxInsert er inserts a frame around the image
%Output image is set to variable 'VideoOut'
    imshow(videoOut,'border','tight');
    %imshow basically displays images
    %parameters 'Border','tight' indicates the compells the images to be
    %displayed with out a border
    f=findobj('tag','webcam');
    %
    if (isempty(f));
        [hueChannel,~,~] = rgb2hsv(frame);
% Display the Hue Channel data and draw the bounding box around the face.
%%figure, imshow(hueChannel), title('Hue channel data');
rectangle('Position',bbox(1,:),'LineWidth',2,'EdgeColor',[1 1 0])
%Creates 2-D rectangle at Position of BBOX with width  and Edgecolor
hold off
%Resets to default behaviour
%Clears existing graphs and resets axis properties to their Defaults
noseDetector = vision.CascadeObjectDetector('Nose');
%Detects nose properties from the video frame using Cascade package
%the properties are assigned to a variable noseDetector
faceImage    = imcrop(frame,bbox);
%crops the Image 'Frame' with Bounding BOX
%%imshow(faceImage)  
%Displays image
noseBBox     = step(noseDetector,faceImage);
%Returns NoseBBOX Matrix
noseBBox(1:1) = noseBBox(1:1) + bbox(1:1);
videoInfo    = info(obj);
ROI=get(obj,'ROI');
%returns the value of Specified property from the Obj image
VideoSize = [ROI(3) ROI(4)];
videoPlayer  = vision.VideoPlayer('Position',[300 300 VideoSize+60]);
%Play video or display image with specified position
tracker = vision.HistogramBasedTracker;
initializeObject(tracker, hueChannel, bbox);
time=0;
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