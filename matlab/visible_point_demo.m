
clear
clc
MAXDIST=40;
sampleing_and_HPR=[2.4 1.3008]
% sampleing_and_HPR=[1 1.9765]
% sampleing_and_HPR=[0 2.1934]

samplingDistance=sampleing_and_HPR(1);
MAX_ITERS = 50;
MINIMUM_OBS = 5;
radiusHPR=sampleing_and_HPR(2);


%camera settings -- Phantom 4
f=3.6;
psize=0.00156192;
sensorWidth=4000;
sensorHeight=3000;
cameraInteriors=struct;
    cameraInteriors.f=f;
    cameraInteriors.cw=sensorWidth*psize;
    cameraInteriors.ch=sensorHeight*psize;
    cameraInteriors.psize=psize;
    cameraInteriors.sensorWidth=sensorWidth;
    cameraInteriors.sensorHeight=sensorHeight;
    %fov=2*atan(W/2f)
    cameraInteriors.fov=2*atand(sensorHeight*psize/2/f);



addpath('C:\Users\shuhang\Documents\GitHub\UAV_planning_photogrammetry\function');
pointCloudPath='D:\Datasets\Hessigheim_Pointcloud\DJI_flight\shiplock_hole_iso - Cloud - aligned.txt';
ptCloud = pointCloud(pointCloudPath);
ptCloud.select('UniformSampling', samplingDistance); % downsample, unit in m
ptCloud.reconstruct;


% EO=[166.813862963580,188.359227692708,257.337104034871,-1.19196060848343,-0.807371930836482,179.501102625869];
EO=[112.813170398118,46.6824481954592,252.118813445092,0,40.8297032543944,154.868355148553];
point_3D_num=[(ptCloud.X),(ptCloud.A.id)]; % [x y z point number]
isInCamFrame=pointInCamFrame(EO, cameraInteriors, point_3D_num);
pointInFrame=point_3D_num(isInCamFrame,:);

camPositions=repmat(EO(1:3), size(pointInFrame,1),1);
camPointVectors=pointInFrame(:,1:3)-camPositions;
distances=vecnorm(camPointVectors')';


fv = read_ply2('shiplock_poisson_mesh_aligned.ply');
v = fv.vertices';
f = fv.faces';
t = opcodemesh(v,f);
[isHit,distanceHit] = t.intersect(camPositions',camPointVectors'); 

indVisible = (distances<=distanceHit+0.05);
indInMaxDist=distances<=ones(numel(distances),1)*MAXDIST;
pointVisible=pointInFrame(and(indVisible, indInMaxDist),:);

obs=getCameraObservation(EO, point_3D_num, cameraInteriors, MAXDIST, radiusHPR, [], 0);

pcshow(point_3D_num(:,1:3));
hold on;
plotPoints(pointVisible,[],'r*' );
plotPoints(point_3D_num,obs(:,3),'bo' );

plotCameras(EO,[0 0 0], 'r');



