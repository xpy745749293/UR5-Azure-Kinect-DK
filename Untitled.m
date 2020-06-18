% ScanPoint = csvread('C:\Users\Administrator\Desktop\ScanPoint.csv');                                        
%read the .csv file

pointcloud_1 = table2array(ScanPoint(1:92160,:));
pointcloud_2 = table2array(ScanPoint(92160+1:92160*2,:));
pointcloud_3 = table2array(ScanPoint(92160*2+1:92160*3,:));
pointcloud_4 = table2array(ScanPoint(92160*3+1:92160*4,:));
pointcloud_5 = table2array(ScanPoint(92160*4+1:92160*5,:));                                                 
%transform the table to array

pc_all = [pointcloud_1;pointcloud_2;pointcloud_3;pointcloud_4;pointcloud_5];
% get all the pointclud

figure(1);
scatter3(pc_all(:,1),pc_all(:,2),pc_all(:,3),5,pc_all(:,4:6)/255,'filled');
% scatter3(pointcloud_1(:,1),pointcloud_1(:,2),pointcloud_1(:,3),5,pointcloud_1(:,4:6)/255,'filled');
title('Original Point Cloud');

%using pc_1 as an example
Data = [pc_all(:,1),pc_all(:,2),pc_all(:,3)];
% Data = [pointcloud_1(:,1),pointcloud_1(:,2),pointcloud_1(:,3)];
pc = pointCloud(Data(:,1:3));
% color_uint8 = uint8(pointcloud_1(:,4:6));
color_uint8 = uint8(pc_all(:,4:6));
pc.Color = color_uint8;

figure(2);
pcshow(pc); %显示点云
title('Original Point Cloud');

pc_denoise=pcdenoise(pc,'NumNeighbors',90,'Threshold',1);
figure(3);
pcshow(pc_denoise); %显示去噪点云
title('Denoised Point Cloud');

minDistance = 0.1;
[labels,numClusters] = pcsegdist(pc_denoise,minDistance);
figure(4);
pcshow(pc_denoise.Location,labels);%显示聚类点云
colormap(hsv(numClusters));
title('Nonground Point Cloud Clusters');
disp(" the number of objects:");
disp(numClusters);

roi = [0.5 1 0 1 -0.7 0.4];
indices = findPointsInROI(pc_denoise,roi);
pc_roi = select(pc_denoise,indices);
figure(5)
% pcshow(pc_denoise.Location,[0.5 0.5 0.5])
% hold on
pcshow(pc_roi.Location);
title('The Plant');
legend('Point Cloud','Points within ROI','Location','southoutside','Color',[1 1 1])
hold off

% pcwrite(ptCloud, 'test.pcd', 'Encoding', 'ascii'); %将程序中的xyz数据写入pcd文件中
% pc = pcread('test.pcd');
