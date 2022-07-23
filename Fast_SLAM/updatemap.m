%mapold function
function mapnew=updatemap(x,y,theta,mapold,lidarranges,lidar_angle,size)


% -----------------------------------------------------------------------------------------------------------------------
%constants
zm=30;
beta=0.0174533;
alpha=0.05;
l=1;
k_min=0;
%size=400;
centroid_x_inertia=ones(size,size);
centroid_y_inertia=ones(size,size);
j=ones(size,size);
i=ones(size,size);

% -----------------------------------------------------------------------------------------------------------------------
% Calculate Two matrices to store the coordinates of the centers

for i = 1:size    
    for j = 1:size
       
        %get centroid
        centroid_x_m=(j*0.1)-0.05;
        centroid_y_m=(i*0.1)-0.05;
       
        % A Matrix that stores ONLY the x coordinates.
        % For every size squares in the x_direction, get their respective x
        % coordinates, and continue to get them column by column.
       
        centroid_x_inertia(i,j)=centroid_x_m-((size/2)*0.1);
       
        % A Matrix that stores ONLY the y coordinates.
        % For every size squares in the y_direction, get the respective y
        % coordinates of their centers, and continue to get them row by row.
 
        centroid_y_inertia(i,j)=centroid_y_m-((size/2)*0.1);
       
        % To understand this, please press on centroid_x_inertia and
        % centroid_y_inertia in the workspace
    end
end

% Reshape to transform Matrices into Vectors
centroid_x_inertia=reshape(centroid_x_inertia.',1,[]);
centroid_y_inertia=reshape(centroid_y_inertia.',1,[]);

% Now store a number representing each square
for m = 1:size
    for n = 1:size
        i(m,n) = m;
        j(m,n) = n;
    end
end

% Reshape to transform to
i=reshape(i.',1,[]);
j=reshape(j.',1,[]);

% -----------------------------------------------------------------------------------------------------------------------

for m=1:(size*size)

        %Calculate the distance r between the point mi and the robot
        r=sqrt((centroid_x_inertia(m) - x)^2+(centroid_y_inertia(m) - y)^2);
        %calculate phi, angle between robot x and r
        phi=atan2(centroid_y_inertia(m)- y,centroid_x_inertia(m) - x)-theta;
       
        %get the index of nearest ray and the value of the angle of this
        %nearest ray with r
        phi_lidar=abs(phi*ones(1,720)-lidar_angle);
        %k_min=find(phi_lidar == min(phi_lidar(:)));%howabout getting two values
        [phi_lidar,k_min]=min(phi_lidar);
       
        %conditions
        %r taller than zmax or ray hit object before raching the cell or
        %closesest ray angle greater than beta/2
        %these are all the cases where I can not reach the cell
        if r>min(zm,lidarranges(k_min)+(alpha/2)) || phi_lidar>(beta/2)
            mapold(i(m),j(m))=mapold(i(m),j(m));
            %l=l+1
        %if I can rach the cell
        %if the closest ray is shorter than zmax and it is very close to r then m is occ    
        elseif lidarranges(k_min)<zm && abs(r-lidarranges(k_min))<alpha/2
            if mapold(i(m),j(m))<100
                mapold(i(m),j(m))=mapold(i(m),j(m))+20; %add 10 to get near to occ near white
                %l=l+1
            end
        %if r shorter than the cclosest ray then it is free
        elseif r<=lidarranges(k_min)
            if mapold(i(m),j(m))<0
                mapold(i(m),j(m))=mapold(i(m),j(m))-10; %subtract 10 to get near to free near black
                %l=l+1
            end
        end                    
end

mapnew=mapold;


