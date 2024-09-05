imaqhwinfo

%% assignment 1 - after installing OS Generic Video Interface
camera_info = imaqhwinfo('winvideo');
% use camera_info = imaqhwinfo('winvideo') on Windows machine

% check camera information
device_id = camera_info.DeviceInfo(2).DeviceID;
default_format = camera_info.DeviceInfo(2).DefaultFormat;
supported_formats = camera_info.DeviceInfo(2).SupportedFormats;

disp(['Device ID: ', num2str(device_id)])
disp(['Default Format: ', default_format])
disp('Supported Formats:')
disp(supported_formats)

% use imaqreset to reset your camera if it starts acting up
imaqreset

video_obj = videoinput('winvideo', device_id, default_format);

video_obj.ReturnedColorSpace = 'rgb';
f = figure('Visible', 'off'); 
vidRes = video_obj.VideoResolution;
imageRes = fliplr(vidRes);
hImage = imshow(zeros(imageRes));
num_images = 15;
for i=1:num_images
    % preview video object
    preview(video_obj, hImage);
    waitforbuttonpress;
    stoppreview(video_obj);
    % start to enable grab
    start(video_obj);
    img = getdata(video_obj);
    img = img(:, :, (1:3));
    imwrite(img, "calib_img"+num2str(i)+".png")
    stop(video_obj);
end

clear video_obj

%% Assignment 2
% After running the Camera Calibrator app, display the matrix of intrinsic
% parameters

cameraParams.Intrinsics.K

%% Assignment 3

% get one image
video_obj = videoinput('winvideo', device_id, default_format);
video_obj.ReturnedColorSpace = 'rgb';
f = figure('Visible', 'off'); 
vidRes = video_obj.VideoResolution;
imageRes = fliplr(vidRes);
hImage = imshow(zeros(imageRes));
% preview video object
preview(video_obj, hImage);
waitforbuttonpress;
stoppreview(video_obj);
% start to enable grab
start(video_obj);
img = getdata(video_obj);
img = img(:, :, (1:3));
stop(video_obj);
clear video_obj
% img = imread('slika1.jpeg');

imshow(img);title('Original image')


%% 

%img = imread('calib_img1.png');
% using the instrinsic parameters we can undistort the image if necessary 
[img_undist,newOrigin] = undistortImage(img,cameraParams,'OutputView','full');

% detect corners in the image
[imagePoints,boardSize] = detectCheckerboardPoints(img_undist);

% set size of a square in milimeters and calculate locations in the checkerboard frame
squareSize = 20;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% calculate extrinsic parameters
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

[orientation_from_pattern, location_from_pattern] = extrinsicsToCameraPose(R, t);

% display location of the camera with respect to the pattern frame
location_from_pattern

%% Assignment 4
squareSize = 19.7;
checkerboard_points = generateCheckerboardPoints(boardSize, squareSize);

% create homogeneous coordinates
checkerboard_points_h = [checkerboard_points zeros(size(checkerboard_points,1),1) ones(size(checkerboard_points,1),1)];

% measure the pose of the pattern in the frame {R}
% ovo T treba popunit
T = []; 

% transform corner points to frame {R}
worldPoints = transpose(T*transpose(checkerboard_points_h));
worldPoints = worldPoints(:,1:2);

[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

[orientation_from_world, location_from_world] = extrinsicsToCameraPose(R, t);

% display position of camera in the {R} frame
location_from_world

%% Assignment 5

% % get one image where camera axis is vertical with respect to the pattern
% video_obj = videoinput('linuxvideo', camera_info.DeviceInfo.DeviceID, camera_info.DeviceInfo.DefaultFormat);
% video_obj.ReturnedColorSpace = 'rgb';
% f = figure('Visible', 'off'); 
% vidRes = video_obj.VideoResolution;
% imageRes = fliplr(vidRes);
% hImage = imshow(zeros(imageRes));
% % preview video object
% preview(video_obj, hImage);
% waitforbuttonpress;
% stoppreview(video_obj);
% % start to enable grab
% start(video_obj);
% img = getdata(video_obj);
% img = img(:, :, (1:3));
% stop(video_obj);
% clear video_obj

img=imread('slika_zad5.jpeg');
imshow(img);title('Original image')
%% 


% using the instrinsic parameters we can undistort the image if necessary 
[img_undist,newOrigin] = undistortImage(img,cameraParams,'OutputView','full');

% detect corners in the image
[imagePoints,boardSize] = detectCheckerboardPoints(img_undist);

% set size of a square in milimeters and calculate locations in the checkerboard frame
squareSize = 19.7;
checkerboard_points = generateCheckerboardPoints(boardSize, squareSize);

% create homogeneous coordinates
checkerboard_points_h = [checkerboard_points zeros(size(checkerboard_points,1),1) ones(size(checkerboard_points,1),1)];

% measure the pose of the pattern in the frame {R}
T = [] 

% transform corner points to frame {R}
worldPoints = transpose(T*transpose(checkerboard_points_h));
worldPoints = worldPoints(:,1:2);

% calculate extrinsic parameters
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

[orientation_from_pattern, location_from_pattern] = extrinsicsToCameraPose(R, t);

% insert processing to extract the colored circle from the image

% extract the centroid of the colored circle 

% using intrinsic camera matrix and distance of the pattern from the camera
% calculate the world coordinates of the colored circle

% z_pattern = % distance of the pattern from the camera
% u = % centroid x value in pixels
% v = % centroid y value in pixels
% cx, cy, fx, fy are instrinsic camera parameters, extract them from the
% matrix
% insert calculation from the lecture slides
x_camera = [];
y_camera = [];
z_camera = [];

% convert to homogeneous coordinates
centroid_camera_h = [x_camera; y_camera; z_camera; 1];

% calculate centroid in world coordinate frame
T_world_camera = [orientation_from_world', location_from_world'; 0 0 0 1];

centroid_world_h = T_world_camera * centroid_camera_h;

% display position of the object in frame {R}
centroid_world = centroid_world_h(1:3,1);

%% Assignment 6

% % get one image with arbitrary orientation of the pattern
% video_obj = videoinput('linuxvideo', camera_info.DeviceInfo.DeviceID, camera_info.DeviceInfo.DefaultFormat);
% video_obj.ReturnedColorSpace = 'rgb';
% f = figure('Visible', 'off'); 
% vidRes = video_obj.VideoResolution;
% imageRes = fliplr(vidRes);
% hImage = imshow(zeros(imageRes));
% % preview video object
% preview(video_obj, hImage);
% waitforbuttonpress;
% stoppreview(video_obj);
% % start to enable grab
% start(video_obj);
% img = getdata(video_obj);
% img = img(:, :, (1:3));
% stop(video_obj);
% clear video_obj

img=imread('slika_zad6.jpeg');
imshow(img);title('Original image')
%% 


% using the instrinsic parameters we can undistort the image if necessary 
[img_undist,newOrigin] = undistortImage(img,cameraParams,'OutputView','full');

% detect corners in the image
[imagePoints,boardSize] = detectCheckerboardPoints(img_undist);

% set size of a square in milimeters and calculate locations in the checkerboard frame
squareSize = 19.7;
checkerboard_points = generateCheckerboardPoints(boardSize, squareSize);

% create homogeneous coordinates
checkerboard_points_h = [checkerboard_points zeros(size(checkerboard_points,1),1) ones(size(checkerboard_points,1),1)];

% measure the pose of the pattern in the frame {R}
T = []

% transform corner points to frame {R}
worldPoints = transpose(T*transpose(checkerboard_points_h));
worldPoints = worldPoints(:,1:2);

% calculate extrinsic parameters
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

% insert processing to extract the colored circle from the image

% extract the centroid of the colored circle 

% u = % centroid x value in pixels
% v = % centroid y value in pixels

% display position of the object in frame {R}
pointsToWorld(cameraParams.Intrinsics, R, t, [u v])

