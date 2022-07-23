function [resampled_cloud,resampled_map]=resample_map_cloud(maps,cloudold,p,n)

%normalize
p=(p/sum(p))*1000;

s=randsample(n,n,true,p)

for i=1:1:n
cloudold(i,:)=cloudold(s(i),:);
maps(:,:,i)=maps(:,:,s(i));
end

resampled_cloud=cloudold;
resampled_map=maps;
end