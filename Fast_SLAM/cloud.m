
% Function that moves each particles acc to the odom motion model 
function cloudnew=cloud(alpha,odomold,odomnew,cloudold)

rot1=atan2(odomnew(2)-odomold(2),odomnew(1)-odomold(1))-odomold(3);
trans=sqrt(((odomnew(1)-odomold(1))^2) + ((odomnew(2)-odomold(2))^2));
rot2=odomnew(3)-odomold(3)-rot1;

for i=1:40
rot1_err=rot1-calcsample(alpha(1)*rot1 + alpha(2)*trans);
trans_err=trans-calcsample(alpha(3)*trans + alpha(4)*(rot1+rot2));
rot2_err=rot2-calcsample(alpha(1)*rot2 + alpha(2)*trans);

cloudnew(i,1)=cloudold(i,1)+trans_err*cos(cloudold(i,3)+rot1_err);
cloudnew(i,2)=cloudold(i,2)+trans_err*sin(cloudold(i,3)+rot1_err);
cloudnew(i,3)=cloudold(i,3)+rot1_err+rot2_err;
end

end