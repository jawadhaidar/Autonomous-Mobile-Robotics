% Code to perform Raycasting on a Supplied Map to return simulated Lidar
% Range values.
% % Author- Shikhar Shrestha, IIT Bhubaneswar





% Input parameters are x-position, y-position, Binary image of map, number
% of rays to cast, Lidar range in cm.
% % Returned Range values are obtained on scanning clockwise. 
% xc,yc in pixels in map frame
%lidarMin and lidarrange in pixels
%starting theta in degrees
function [angle,range]=castrays(xc,yc,map,n,lidarMin,lidarrange,thetastart,DEBUG)
    %get the size , 200 x 200 pixels
    MatrixSize = size(map);
    angle = [];
    %xc =  MatrixSize(2) - xc; 
    %to put the axis at the bottom left
    yc = MatrixSize(1) - yc
    if(xc<=0) 
        xc = 1;
    end
    if(yc<=0) 
        yc = 1 ;
    end


    % Conversion factor from pixels to centimeters.
    pixeltom=0.1;
    %floor() round
    %in case you want to put the pt in a black box , 1 is black, get out of
    %code
    if map(floor(yc),floor(xc))==1
        range=zeros(n,1);
        return;
    end

    %debug means draw to check
    if DEBUG == 1
    figure(2);
    hold off;
    imshow(map);
    hold on;
    plot(xc,yc,'b*');
    end

    % if nargin==3
    %     n=20;
    %     lidarrange=500;
    % end
    % 
    % if nargin==4
    % lidarrange=500;
    % end
    %inistialize range and angles
    range=zeros(n,1);
    angle = zeros(n,1);

    %step angle in degrees

    %thetastep=360/n;
    %starting theta in degrees
    %thetastart=200;
    %lidar angle range in degrees/n
    thetastep=270/n;

    for i=1:n
      if i==1
          i
           %pause(0.01)
          theta=-thetastart;
          r=linspace(lidarMin,lidarrange,2000);
          x=xc+(r*cosd(theta));
          y=yc+(r*sind(theta));
      else

     %pause(0.01)
     %why negative
    theta=thetastep*(i-1)-thetastart;
      end
    %range of 2000 values along r (theta) as if you discretized r to 2000 pt
    %interms of pixels
    r=linspace(lidarMin,lidarrange,2000);
    x=xc+(r*cosd(theta));
    y=yc+(r*sind(theta));

    % Removing points out of map
    temp=[];
    %numel number of elements in an array
    %> means outide the map
    for k=1:numel(x)
        if x(k)>MatrixSize(2) || y(k)>MatrixSize(1) || x(k)<=0 || y(k)<=0
            %fill temp with unvalid k that should be droped
            temp=[temp;k];
        end
    end
    %remove unvalid pts
    x(temp)=[];
    y(temp)=[];

    %draw
    if DEBUG == 1 
    figure(2);
    plot(x,y,'r');
    end

    % Computing Intersections
    %rounding
    xint=round(x);
    yint=round(y);

    %Correcting zero map indexing
    %did not get until  now
    for l=1:numel(xint)
        if xint(l)==0
            xint(l)=1;
        end
        if yint(l)==0
            yint(l)=1;
        end
    end

    b=[];
    for j=1:numel(xint)
    %fill map with valid values for the line at a certain angle
    b=[b;map(yint(j),xint(j))];
    end
    %find the index black box
    ind=find(b==1);

    %if there is a black box ,choose first one
    if ~isempty(ind)
    xb=x(ind(1));
    yb=y(ind(1));

    %draw this pt as green dot
    if DEBUG == 1
    figure(2);
    plot(xb,yb,'g*');
    end
    %calculate distance from blue to green in pixels
    dist=sqrt((xc-xb).^2 + (yc-yb).^2);
    range(i)=dist;
    %return angle in radian
    angle(i)=-theta;
    end
    %pause(0.000001);
    end
    %Converting to m from pixels.
    range=range*1*pixeltom;
    %range=flipud(range);
    angle=angle';
    
    hold off