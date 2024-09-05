

camera_info = imaqhwinfo('winvideo');
% use camera_info = imaqhwinfo('winvideo') on Windows machine

% check camera information
device_id = camera_info.DeviceInfo(2).DeviceID;
default_format = camera_info.DeviceInfo(2).DefaultFormat;
supported_formats = camera_info.DeviceInfo(2).SupportedFormats;

disp(['Device ID: ', num2str(device_id)])
disp(['Default Format: ', default_format])
%disp('Supported Formats:')
%disp(supported_formats)

% use imaqreset to reset your camera if it starts acting up
%imaqreset

video_obj = videoinput('winvideo', device_id, default_format);

video_obj.ReturnedColorSpace = 'rgb';
f = figure('Visible', 'off'); 
vidRes = video_obj.VideoResolution;
imageRes = fliplr(vidRes);
hImage = imshow(zeros(imageRes));
clear video_obj

% After running the Camera Calibrator app, display the matrix of intrinsic
% parameters
load('kamera_sesija_69.mat');
cameraParams = calibrationSession.CameraParameters;
% cameraParams.Intrinsics.K
% K = 1.0e+03 * [
%     1.1234, 0, 0.5408;
%     0, 1.1220, 0.3222;
%     0, 0, 0.0010
% ];

%---------------------------------
% slika s kojom se obavlja postavljenje kamere u koordinatni sustav
img = imread('calib_img1.png');
% using the instrinsic parameters we can undistort the image if necessary 
[img_undist,newOrigin] = undistortImage(img,cameraParams,'OutputView','full');

% detect corners in the image
[imagePoints,boardSize] = detectCheckerboardPoints(img_undist);

% set size of a square in milimeters and calculate locations in the checkerboard frame
squareSize = 19.8;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% calculate extrinsic parameters
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

[orientation_from_pattern, location_from_pattern] = extrinsicsToCameraPose(R, t);

% lokacija kamere s obzirom na (0,0) papira za kalibraciju u slici img1.png
%location_from_pattern

checkerboard_points = generateCheckerboardPoints(boardSize, squareSize);

% create homogeneous coordinates
checkerboard_points_h = [checkerboard_points zeros(size(checkerboard_points,1),1) ones(size(checkerboard_points,1),1)];

% measure the pose of the pattern in the frame {R}
% --------------------------------------------------
% ovo  -> poza sustava checkboarda u sustavu nase ruke (sredina donjeg ruba podloge)
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
%location_from_world
%% 


% ------------------------------------------------------
% iznad linije uvijek pokrecete, to je kalibracija sa slikom img1.png
% tj. slika checkboarda na podlozi, ako se pozicija kamere promijenila
% a vjv je jer ju svako malo neko nastimava,
% treba s kamere uslikat novu sliku kao i img1.png (dakle sa checkboardom polozenim na plocu)

% dvije opcije, odabrat onu koja daje bolje rezultate koordinata


% PRVA
% umjesto KOORDINATE_TOCKE saljete [x, y] u pikselima iz slike
% dakle x,y putanje
%-----------------------------------------
lista_tocaka = KOORDINATE_TOCKE;
BR_TOCAKA = size(lista_tocaka, 1);

% cx, cy, fx, fy are instrinsic camera parameters, extract them from the
% matrix
cx = cameraParams.Intrinsics.K(1, 3);
cy = cameraParams.Intrinsics.K(2, 3);
fx = cameraParams.Intrinsics.K(1, 1);
fy = cameraParams.Intrinsics.K(2, 2);

% calculate centroid in world coordinate frame
T_world_camera = [orientation_from_world', location_from_world'; 0 0 0 1];

for i = 1:BR_TOCAKA
    centroid = lista_tocaka(i,:);
    % z_pattern = % distance of the pattern from the camera
    z_pattern = location_from_pattern(3); % MOZDA TREBA location_from_world(1)
    % nisam sig
      
    % u = % centroid x value in pixels
    u = centroid(1);
    % v = % centroid y value in pixels
    v = centroid(2);

    % insert calculation from the lecture slides
    x_camera = [ (z_pattern*(u - cx)) / fx ];
    y_camera = [ (z_pattern*(v - cy)) / fy ];
    z_camera = [ z_pattern ];
   
    % convert to homogeneous coordinates
    centroid_camera_h = [x_camera; y_camera; z_camera; 1];
    
    centroid_world_h = T_world_camera * centroid_camera_h;
    
    % display position of the object in frame {R}
    centroid_world = centroid_world_h(1:3,1);
    % msm da bi u ^ovoj varijabli trebo bit rezultat

    centroid_world_list(i,:) = centroid_world;
end

RJESENJE = centroid_world_list;

% % DRUGA
% %------------------------------------------------------
% % extract the centroid of the colored circle 
% centroid = KOORDINATE_TOCKE;
% 
% % u = % centroid x value in pixels
% % v = % centroid y value in pixels
% u = centroid(1);
% v = centroid(2);
% 
% % display position of the object in frame {R}
% rezultat = pointsToWorld(cameraParams.Intrinsics, R, t, [u v]);
% % u ^ovoj varijabli bi trebo bit rezultat


%---------------------------------------------------
% preporucam da opciju koju odaberete stavite u for petlju i izvrtite
% za svih 20, 50, 100 il kolko vec tocaka putanje, i onda to spremite u
% neku listu [x,y,z] koordinata (z bi trebo bit uvijek isti) koje onda
% redom saljete na racunanje inverzne kinematike, pa postavljate
% vrijednosti zglobova na dobivene rezultate

% na taj nacin se onda ovaj potprogram moze samo jednom pokrenut nakon
% izvrsavanja yolo detekcije, ne treba konstantno za svaku tocku kao npr
% kad se svaki put mijenja inverzna il direktna kin