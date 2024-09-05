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
clear video_obj

%% Assignment 2
% After running the Camera Calibrator app, display the matrix of intrinsic
% parameters
load('kamera_sesija_3.mat')
cameraParams = calibrationSession.CameraParameters;
cameraParams.Intrinsics.K

%% Assignment 3

% get one image
% video_obj = videoinput('winvideo', device_id, default_format);
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
% imwrite(img, "img1.png")
% stop(video_obj);
% clear video_obj
% % img = imread('slika1.jpeg');
% 
% imshow(img);title('Original image')


%% 
img = imread('calib_img1.png');
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

% lokacija kamere s obzirom na (0,0) papira za kalibraciju u slici img1.png
location_from_pattern

%% Assignment 4
squareSize = 20;
checkerboard_points = generateCheckerboardPoints(boardSize, squareSize);

% create homogeneous coordinates
checkerboard_points_h = [checkerboard_points zeros(size(checkerboard_points,1),1) ones(size(checkerboard_points,1),1)];

% measure the pose of the pattern in the frame {R}
% ovo T treba popunit
T = [1 0 0 -75;
     0 -1 0 317;
     0 0 -1 0;
     0 0 0 1]; 

% transform corner points to frame {R}
worldPoints = transpose(T*transpose(checkerboard_points_h));
worldPoints = worldPoints(:,1:2);

[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

[orientation_from_world, location_from_world] = extrinsicsToCameraPose(R, t);

% lokacija kamera s obzirom na ishodište koordinatnog sustava (sredina donjeg brida ravne ploče)
% +x udesno
% +y naprijed
% -z u vis gore
location_from_world
