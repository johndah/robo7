
%  opts       - parameters (struct or name/value pairs)
%   (1) features and model:
%   .pPyramid   - [{}] params for creating pyramid (see chnsPyramid)
%   .filters    - [] [wxwxnChnsxnFilter] filters or [wFilter,nFilter]
%   .modelDs    - [] model height+width without padding (eg [100 41])
%   .modelDsPad - [] model height+width with padding (eg [128 64])
%   .pNms       - [..] params for non-maximal suppression (see bbNms.m)
%   .stride     - [4] spatial stride between detection windows
%   .cascThr    - [-1] constant cascade threshold (affects speed/accuracy)
%   .cascCal    - [.005] cascade calibration (affects speed/accuracy)
%   .nWeak      - [128] vector defining number weak clfs per stage
%   .pBoost     - [..] parameters for boosting (see adaBoostTrain.m)
%   .seed       - [0] seed for random stream (for reproducibility)
%   .name       - [''] name to prepend to clf and log filenames
%   (2) training data location and amount:
%   .posGtDir   - [''] dir containing ground truth
%   .posImgDir  - [''] dir containing full positive images
%   .negImgDir  - [''] dir containing full negative images
%   .posWinDir  - [''] dir containing cropped positive windows
%   .negWinDir  - [''] dir containing cropped negative windows
%   .imreadf    - [@imread] optional custom function for reading images
%   .imreadp    - [{}] optional custom parameters for imreadf
%   .pLoad      - [..] params for bbGt>bbLoad (see bbGt)
%   .nPos       - [inf] max number of pos windows to sample
%   .nNeg       - [5000] max number of neg windows to sample
%   .nPerNeg    - [25]  max number of neg windows to sample per image
%   .nAccNeg    - [10000] max number of neg windows to accumulate
%   .pJitter    - [{}] params for jittering pos windows (see jitterImage)
%   .winsSave   - [0] if true save cropped windows at each stage to disk

%% images for training
clear;
cd(fileparts(which('acfMain.m'))); dataDir='../../data/';

%% sets up opts for training detector
opts=acfTrainCustom(); 
% modelDs    [h w]
opts.modelDs=[50 50]; opts.modelDsPad=[60 60];

opts.pPyramid.pChns.pColor.smooth=0; 
opts.pPyramid.pChns.pGradHist.softBin=1;
opts.pPyramid.pChns.shrink=2;

opts.pPyramid.pChns.pColor.colorSpace='luv';
opts.pPyramid.pChns.pColor.enabled=1;
opts.pPyramid.pChns.pGradHist.nOrients=4;
opts.pPyramid.pChns.pGradHist.enabled=0;
opts.pPyramid.pChns.pGradMag.colorChn=1;
opts.pPyramid.pChns.pGradMag.enabled=1;

opts.pBoost.pTree.maxDepth=5; opts.pBoost.discrete=0; %[1] train Discrete-AdaBoost or Real-AdaBoost
opts.pBoost.pTree.fracFtrs=1/16; opts.pBoost.verbose=1;

opts.nWeak=[64 128 256];
opts.nPos = inf; opts.nNeg=8000; opts.nAccNeg=20000; opts.nPerNeg=5;
opts.posGtDir=[dataDir 'Train' '/annotation'];
opts.posImgDir=[dataDir 'Train' '/image'];
% opts.negImgDir=[dataDir 'Train' '/NEG_SAMPLES_640_360'];
%opts.posWinDir = [dataDir 'pos'];
% opts.negWinDir = [dataDir 'Test' '/NEG_SAMPLES_640_360'];

opts.pJitter=struct('flip',0);
% opts.pPyramid.lambdas = [0, 0.1025];
% opts.winsSave = 1;

opts.name='models/Acf+'; 
% lbls          - object id
% squarify      - format=2, use original w, alter h
%                 format=3, use original h, alter w
%                 format=4: preserve area, alter w and h
%      gith           format=5, original bbs
% arRng         - aspect ratio range
pLoad = {'lbls', 'p', 'squarify', {4, 0}, 'arRng', [0.6 1.6], 'wRng', [40, 300]};
opts.pLoad = [pLoad 'format', 3, 'rescale', [480, 640]];

% if(1), opts.filters = [5 4]; opts.name='models/Ldcf+'; end

%% train detector
detector = acfTrainCustom( opts );

%% run detector on a sample image (see acfDetect)
% imgNms=bbGt('getFiles',{[dataDir 'Eval' '/POS_SAMPLES_640_360/'], [dataDir 'Eval' '/LABELS/']});
I=imread('2.png'); 
tic, bbs=acfDetect(I,detector); toc
% bbs = bbs(1,:);
figure; im(I); bbApply('draw',bbs); pause(.1);

%% run detector on several images
% imgNms=bbGt('getFiles',{[dataDir 'Train' '/image/']});
% Nums=size(imgNms,2);
% threshold = 0;
% for i = 1:20
%     figure(i);
%     ind=randi(Nums);
%     I=imread(imgNms{ind}); %I=imresize(I, [360 640]);
% %     I = imread('test_3.png'); I=imresize(I, [360 640]);
% %      I = I(120:240,481:end,:);
% %     I=padarray(I,[10,10],'replicate','both');
%     tic, bbs=acfDetect(I,detector); toc
%     
% %      bbs = bbs(1,:);
% %     subplot(5, 4, i);
%     bbs = bbs(bbs(:,5)>threshold,:);
%     imshow(I); bbApply('draw',bbs);
%     pause(.1);
% end
% 
 %% run detector and create a video
% name = {'lab_maze'; 'lab_maze2'; 'light'};
% for j = 3
%     imgNms=bbGt('getFiles',{[dataDir 'Train/image/' name{j}]});
%     outputVideo = VideoWriter(name{j});
%     outputVideo.FrameRate = 10;
%     open(outputVideo)
% 
%     for i = 1:length(imgNms)
%         fh = figure(10);
%         I=imread(imgNms{i}); 
% %         Ipad=padarray(I,[10,10],'replicate','both');
% %         bbs=acfDetect(Ipad,detector);
%         %bbs = bbs(1,:);
% %         bbs(:,1:2) = bbs(:,1:2) - 10;
%         bbs=acfDetect(I,detector);
%         imshow(I); bbApply('draw',bbs);
%         frm = getframe(fh);
% %         pause(.1);
%         writeVideo(outputVideo, frm);
%     end
%     close(outputVideo);
% end

%%
name = {'test'};
for j = 1
    imgNms=bbGt('getFiles',{[dataDir 'Test/image/' name{j}]});
    outputVideo = VideoWriter(name{j});
    outputVideo.FrameRate = 10;
    open(outputVideo)

    for i = 1:length(imgNms)
        fh = figure(10);
        I=imread(imgNms{i}); 
%         Ipad=padarray(I,[10,10],'replicate','both');
%         bbs=acfDetect(Ipad,detector);
        %bbs = bbs(1,:);
%         bbs(:,1:2) = bbs(:,1:2) - 10;
        bbs=acfDetect(I,detector);
        imshow(I); bbApply('draw',bbs);
        frm = getframe(fh);
%         pause(.1);
        writeVideo(outputVideo, frm);
    end
    close(outputVideo);
end

% % 
% %% test detector and plot roc (see acfTest)
% pLoad = {'lbls', '0', 'squarify', {5, 0}};
% pLoad = [pLoad 'format', 3, 'rescale', [360, 640]];
% [IoU, Precision, Recall, accRL] = acfTestCustom('name',opts.name,'imgDir',[dataDir 'Eval/POS_SAMPLES_640_360'],...
%   'gtDir',[dataDir 'Eval/LABELS'],'pLoad',pLoad, 'thr', 0.3, 'show',2, 'accLeftRight', 1);
