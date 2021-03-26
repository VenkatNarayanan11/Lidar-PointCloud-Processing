%% Lidar Point Cloud Processing
%% 1) Visualizing the 3D Point Cloud

% Reading the point cloud file
open_file = fopen("002_00000000.bin");
read_file = fread(open_file, 'float32');
count = 1;
for value=1:4:length(read_file)
    x(count) = read_file(value);
    y(count) = read_file(value+1);
    z(count) = read_file(value+2);
    I(count) = read_file(value+3);
    count = count + 1;
end

% Plotting the point cloud
figure(1)
Pt_cloud = pointCloud([x(:),y(:),z(:)],"Intensity",I(:));
pcshow(Pt_cloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Point Cloud colored by its reflective value')
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'3D Point Cloud colored by its reflective value.png'); % save as .png file


%% 2) Downsampling the 3D Point Cloud

% Applying box grid filter to the original point cloud
pt_cloud_down = pcdownsample(Pt_cloud,"gridAverage",1.1);

% Plotting the points
figure(2)
pcshow(pt_cloud_down);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Downsampled 3D Point Cloud to 3D Voxel space points') 
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'Downsampled 3D Point Cloud to 3D Voxel space points.png'); % save as .png file


%% 3) MSAC Algorithm

% Using MSAC to find the ground plane model.

maxDistance = 0.5;
referenceVector = [0,0,1]; 
maxAngularDistance = 5;

% Calculating time complexity of the MSAC algorithm
tic;
[model, inlierIndices, outlierIndices, meanError] = pcfitplane(pt_cloud_down,...
                            maxDistance, referenceVector, maxAngularDistance);
toc;

disp("The model parameters are:" )
disp(model.Parameters)
% Plotting the ground plane model
figure(3)
pcshow(pt_cloud_down);
hold on 
model.plot;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Visualizing ground plane with points in 3D') 
hold off
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'Visualizing ground plane with points in 3D.png'); % save as .png file


% Plotting the off ground points
figure(4)
off_ground_points = select(pt_cloud_down, outlierIndices);
pcshow(off_ground_points);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Visualizing the off ground points in 3D') 
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'Visualizing the off ground points in 3D.png'); % save as .png file


% Plotting the on ground points
figure(5)
on_ground_points = select(pt_cloud_down, inlierIndices);
pcshow(on_ground_points);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Visualizing the on ground points in 3D') 
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'Visualizing the on ground points in 3D.png'); % save as .png file


%% 4) X-Y Projection to the off ground points

off_ground = off_ground_points.Location;
Intensity = off_ground_points.Intensity;
figure(6)
color = Intensity(:); % Intensity values to color
scatter(off_ground(:,1), off_ground(:,2), 0.5, color);
set(gca, 'color', 'k'); % Black Background
xlabel('X');
ylabel('Y');
title('X-Y projection to the off ground points') 
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'X-Y projection to the off ground points.png'); % save as .png file


%% 5) Raw Point Cloud date in Polar Coordinate

% Transforming the pointcloud from cartesian to polar coordinate
[theta, rho, elevation] = cart2pol(x(:),y(:),z(:));

% Pointcloud object for polar coordinate
Polar_ptCloud = pointCloud([theta(:),rho(:),elevation(:)], ...
                 "Intensity",I(:));

% Visualizing pointcloud in polar coordinate             
figure(7)             
pcshow(Polar_ptCloud);
title('Raw Point Cloud in Polar Coordinate')
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'Raw Point Cloud in Polar Coordinate.png'); % save as .png file

% Visualizing the 2D depth image with intensity values as elevation
% X-Y Plane
figure(8);
pointcloud_2D = pointCloud([x(:),y(:),z(:)],'Intensity',z(:));   
pcshow(pointcloud_2D);
view(0,90);
title('2D depth image in X-Y plane');
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'2D depth image in X-Y plane.png'); % save as .png file



% Y-Z plane
figure(9);
pointcloud_2D = pointCloud([x(:),y(:),z(:)],'Intensity',z(:));   
pcshow(pointcloud_2D);
view(90,0);
title('2D depth image in Y-Z plane');
zoom(7)
set(gcf, 'InvertHardCopy', 'off');
saveas(gcf,'2D depth image in Y-Z plane.png'); % save as .png file