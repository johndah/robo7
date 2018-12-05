clear;
cd(fileparts(which('dataProperty.m'))); dataDir='../../data/';

posGtDir=[dataDir 'Train' '/image/'];
posImgDir=[dataDir 'Train' '/annotation/'];
fs={posImgDir,posGtDir};

fs=bbGt('getFiles',fs); nImg=size(fs,2); assert(nImg>0);

%% Aspect ratio, w/h & size of object
r = zeros(14, nImg);
s = zeros(28, nImg);
acc = ones(14, 1);
for i = 1:nImg
  fId=fopen(fs{1,i});
  if(fId==-1), error(['unable to open file: ' fName]); end
  frmt='%f %f %f %f %f';
  in=textscan(fId,frmt);
  in=cell2mat(in);
  
  for j = 1:size(in, 1)
    r(in(j,1)+1, acc(in(j,1)+1)) = in(j, 4) * 640 / in(j, 5) / 480;
    s(2*in(j,1)+1:2*in(j,1)+2, acc(in(j,1)+1)) = [in(j, 4); in(j, 5)];
    acc(in(j,1)+1) = acc(in(j,1)+1) + 1;
  end
  fclose(fId);
end

%% boxplot of aspect ratio
% r = [r0, r1, r2, r3, r4, r5]';
% label = [repmat('0',length(r0),1);
%          repmat('1',length(r1),1);
%          repmat('2',length(r2),1);
%          repmat('3',length(r3),1);
%          repmat('4',length(r4),1);
%          repmat('5',length(r5),1);];
% boxplot(r,label);

%% histogram of aspect ratio
for i = 1:14
    r_i = r(i,:); r_i(r_i==0) = [];
    subplot(4,4,i)
    hist(r_i,100);
    title(['object ', num2str(i-1), ' (max:', num2str(max(r_i)) , ' min:', num2str(min(r_i)), ')' ]);
end

%% Overall aspect ratio
r_all = r;
r_all(r_all==0) = [];
hist(r_all,100);
title(['all objects ', ' (max:', num2str(max(r_all)) , ' min:', num2str(min(r_all)), ')' ]);

%% visualize size
for i = 1:14
    subplot(4,4,i)
    s_i = s(2*i-1:2*i, :);
    s_i(s_i==0) = [];
    s_i = reshape(s_i, [2, length(s_i)/2]);
    x = s_i(1,:)*640;
    y = s_i(2,:)*480;
    scatter(x, y, 'MarkerFaceColor','b','MarkerEdgeColor','b',...
        'MarkerFaceAlpha',.01,'MarkerEdgeAlpha',.01);
    hold on;
    p = polyfit(x,y,1);
    plot(x, polyval(p, x));
    title(['Object ', num2str(i-1), ' (num: ', num2str(size(s_i, 2)), ')']);
    xlabel('width');
    ylabel('height');
end

%% Overall size
s_all = [];
for i = 1:14
    s_i = s(2*i-1:2*i, :);
    s_i(s_i==0) = [];
    s_i = reshape(s_i, [2, length(s_i)/2]);
    s_all = [s_all, s_i];
end
x = s_all(1,:)*640;
y = s_all(2,:)*480;
scatter(x, y, 'MarkerFaceColor','b','MarkerEdgeColor','b',...
    'MarkerFaceAlpha',.01,'MarkerEdgeAlpha',.01);
hold on;
p = polyfit(x,y,1);
plot(x, polyval(p, x));
title(['All objects ', ' (num: ', num2str(size(s_all, 2)), ')']);
xlabel('width');
ylabel('height');