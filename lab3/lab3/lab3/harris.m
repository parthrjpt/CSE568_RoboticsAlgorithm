function [points] = harris(image)
%HARRIS Summary of this function goes here
%   Detailed explanation goes here

    img=double(image);
    %filter = double([-1 0 1; -1 0 1; -1 0 1]);
    filter = fspecial('sobel');
    f_x = filter;
    f_y = rot90(filter);

    [w_in,h_in]=size(img);

    dx =conv2(img,f_x);
    dy=conv2(img,f_y);
    dxy=dx.*dy;
    %order 2 derivative
    filter2=fspecial('gaussian',[3,3],1);
    dx2=conv2(dx.^2,filter2);
    dy2=conv2(dy.^2,filter2);
    dxy2= conv2(dxy.^2,filter2);
    op_kps = zeros([w_in,h_in]);
    for i =1:w_in
        for j=1:h_in
            hmat=[[dx2(i,j) dxy2(i,j)]; [dxy2(i,j) dy2(i,j)]];
            r=det(hmat)-0.04*trace(hmat)^2;
            %fprintf('%d %d \n',i,j);
            op_kps(i,j)=r;
        end
    end
    %disp(op_kps);
    [x,y]=find(op_kps>0,200);
    points =[x,y];
end

