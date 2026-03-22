%% Estimate homogenous transformation.

%Load list of robot poses
load("final!.mat", 'allMat');


images = imageDatastore("imgs");
num = numel(images.Files);
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);
squareSize = .020;
worldPoints = patternWorldPoints("checkerboard",boardSize,squareSize);

I = readimage(images, 1);
imageSize = [size(I,1) size(I,2)];

focalLength = [643.159 643.159];
principalPoint = [650.398 364.779];
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

%intinsics = cameraParams.Intrinsics;
%robotTForm = [0 0 1 15; 0 -1 0 0; 1 0 0 27.28; 0 0 0 1];
%robotTForm = [robotTForm; [0.0712 0 0.9975 20; 0 -1 0 0; 0.9975 0 -.0712 25.5; 0 0 0 1]];
%robotTForm = [robotTForm; [0.3429 0.4472 0.8261 20; 0.1714 -.8944 .413 10; .9236 0 -.3834 20; 0 0 0 1]];
%robotTForm = [robotTForm; [-.2586 0 .966 10; 0 -1 0 0; .966 0 0.2586 30; 0 0 0 1]];
%robotTForm = [robotTForm; [.0189 -.3162 0.9485 15; -.0063 -.9487 -.03162 -5; .9998 0 -.0199 27; 0 0 0 1]];
%robotTForm = [robotTForm; [.638 0 0.7701 26; 0 -1 0 0; .7701 0 -.638 11; 0 0 0 1]];
%robotTForm = [robotTForm; [.3772 -.1789 .9087 22; -.0686 -.09839 -.1652 -4; 0.9236 0 -.3834 20; 0 0 0 1]];
%robotTForm = [robotTForm; [-.0648 0 0.9979 13; 0 -1 0 0; .9979 0 0.0648 28; 0 0 0 1]];
%robotTForm = [robotTForm; [-.055 0.1961 0.979 15; -.011 -.9806 .1958 3.0; .9984 0 0.0561 28; 0 0 0 1]];
%robotTForm = [robotTForm; [.1628 -.356 .9202 21; -.062 -.9345 -.03506 -8; 0.9847 0 -.1742 23; 0 0 0 1]];
%robotTForm = [robotTForm; [.884 .124 .4508 24; 0.1105 -.9923 0.0563 3.0; 0.4543 0 -.8909 0; 0 0 0 1]];
%robotTForm = [robotTForm; [-.2136 -.1521 .965 13; .0329 -.9884 -.1485 -2; .9764 0 .2161 30; 0 0 0 1]];
%robotTForm = [robotTForm; [0.2423 .1961 .9502 20; .0485 -.9806 .19 4; .969 0 -.2471 23; 0 0 0 1]];
%robotTForm = [robotTForm; [-.109 -.0587 .9923 17; .0064 -.9983 -.0584 -1; .994 0 0.1092 28.5; 0 0 0 1]];

%
%robotTForm = [0 0 1 13; 0 -1 0 0; 1 0 0 25.28; 0 0 0 1];
%robotTForm = [robotTForm; [0.07117 0 .99746 17.85765; 0 -1 0 0; .99746 0 -.07117 23.50507; 0 0 0 1]];
%robotTForm = [robotTForm; [.34289 -.44721 0.82609 17.52536; -.17145 -.89443 -.41305 -8.76268; .9236 0 -.38337 18.15281; 0 0 0 1]];
%robotTForm = [robotTForm; [-.25864 0 0.96597 8.51728; 0 -1 0 0; .96597 0 0.25864 28.06805; 0 0 0 1]]
%robotTForm = [robotTForm; [.0189 -.31623 .9485 13.06484; -.0063 -.94858 -.31617 -4.35495; .9998 0 -.01992 25.0004; 0 0 0 1]];
%robotTForm = [robotTForm; [.0189 .31623 .9485 13.06484; 0.0063 -.94868 .31617 4.35495; .9998 0 -.01992 25.0004; 0 0 0 1]];
%robotTForm = [robotTForm; [.63795 0 0.77007 22.72409; 0 -1 0 0; .77007 0 -.63795 9.45985; 0 0 0 1]];
%robotTForm = [robotTForm; [.37718 -.17889 0.9087 19.2779; -.06858 -.98387 -.16522 -3.50507; .9236 0 -0.38337 18.15281; 0 0 0 1]];
%robotTForm = [robotTForm; [-.06476 0 0.9979 11.12591; 0 -1 0 0; .9979 0 0.06476 26.0042; 0 0 0 1]];
%robotTForm = [robotTForm; [-.05503 0.19612 .97904 13.14889; -.01101 -.98058 .19581 2.62978; .99842 0 0.05612 26.00315; 0 0 0 1]];
%robotTForm = [robotTForm; [.16279 -.356 .9202 18.80545; -.06201 -.93449 -.35055 -7.16398; .98471 0 -.1742 21.03058; 0 0 0 1]];
%robotTForm = [robotTForm; [.88399 .12403 .45076 20.24747; .1105 -.99228 0.05634 2.53093; 0.45426 0 -.89087 -.90853; 0 0 0 1]];
%robotTForm = [robotTForm; [-.21361 -.15206 .96501 11.45048; .03286 -.98837 -.14846 -1.76161; .97637 0 0.21613 28.04727; 0 0 0 1]];
%robotTForm = [robotTForm; [.24233 .19612 .95017 17.55418; .04847 -.98058 .19003 3.51084; .96898 0 -.24713 21.06203; 0 0 0 1]];
%robotTForm = [robotTForm; [-.10902 -.05872 .9923 15.22148; .00641 -.99827 -.05837 -.89538; .99402 0 .1092 26.51196; 0 0 0 1]];

robotTForm = allMat;
%num of images, could get this automatically but whatevah
n = 15;
camExtrinsics(n, 1) = rigidtform3d;
rigrobTForm(n, 1) = rigidtform3d;
config = "moving-camera"
for i = 1:n
    calibrationImage = readimage(images, i);
    [undistortedImage, newIntrinsics] = undistortImage(calibrationImage, intrinsics);

    [imagePoints, patternDims] = detectCheckerboardPoints(undistortedImage, PartialDetections=false);
    newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
    imagePoints = imagePoints+newOrigin;
    figure(i)
    imshow(undistortedImage)
    axis on
    hold on;
    plot(imagePoints(:, 1), imagePoints(:, 2), 'ro', 'MarkerSize', 3)
    shape = size(worldPoints)
    hold off;

    for j = 1:shape(1)
        text(imagePoints(j, 1), imagePoints(j, 2), 'p:' + string(j))
        %text(imagePoints(j, 1), imagePoints(j, 2), txt = string(worldPoints(j, 1)) + ", " + string(worldPoints(j, 2)))
    end
    worldPoints = patternWorldPoints("checkerboard", patternDims, squareSize);
    extrinsics = estimateExtrinsics(imagePoints, worldPoints, intrinsics)
    camExtrinsics(i) = extrinsics;
end

for i = 0:n-1
    a = double(robotTForm((i*4)+1:(i*4)+4, 1:4))
    R_recorded = a(1:3, 1:3);
    T_recorded = a(1:3, 4);
    %T_recorded(2) = -1*T_recorded(2);
    T_recorded_meters = .00254*T_recorded;
    %R_recorded(1, 2) = R_recorded(1, 2)*-1;
    %R_recorded(2, 1) = R_recorded(2, 1)*-1;
    %R_recorded(2, 3) = R_recorded(2, 3)*-1;
    R_new = quat2rotm(quatnormalize(rotm2quat(R_recorded)))
    det(R_new);
    %R_new = U*V';
    Tform = inv([R_new T_recorded_meters; 0 0 0 1]);
    rigrobTForm(i+1) = rigidtform3d(Tform);
end

cameraToEndEffectorTform = estimateCameraRobotTransform(camExtrinsics, rigrobTForm, config)