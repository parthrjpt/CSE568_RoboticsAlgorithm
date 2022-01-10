img_location = '../MATLAB/lab3/inputs/image'; % ** will match all the subfolders
img_op_location ='../MATLAB/lab3/';
for i=1:6
   % disp(strcat(img_location,int2str(i),'.jpg'));
    img = imread(strcat(img_location,int2str(i),'.jpg'));
%    imshow(img)
    img_size=size(img);
    break_interval=round((img_size)/3);
    
    bluelength = break_interval;
    greenlength =2*break_interval;
    redlength = 3*break_interval;
    blue=img(1:bluelength,:);
    green=img(bluelength+1:greenlength,:);
    red=img(greenlength+1:redlength,:);
    
    rgbimage(:,:,1)=red;
    rgbimage(:,:,2)=green;
    rgbimage(:,:,3)=blue;
    imwrite(rgbimage,strcat("image",int2str(i),'-color.jpg'),"jpg");
    clear rgbimage;
    
    [offset_rg,offset_br]=im_align1(red,green,blue);
    red_ssd=red;
    green_ssd = circshift(green,offset_rg);
    blue_ssd = circshift(blue,offset_br);
    rgb_ssd(:,:,1) = red_ssd;
    rgb_ssd(:,:,2) = green_ssd;
    rgb_ssd(:,:,3) = blue_ssd;
    imwrite(rgb_ssd,strcat("image",int2str(i),'-ssd.jpg'),"jpg");
    fprintf("image%d: ssd align offset between red and green: [%d,%d] \n",i,offset_rg(1),offset_rg(2));
    fprintf("image%d: ssd align offset between red and blue: [%d,%d] \n",i,offset_br(1),offset_br(2));
    clear rgb_ssd

    [offset_bg,offset_br]=im_align2(red,green,blue);
    %[red_ssd,green_ssd,blue_ssd] = im_align1(red,green,blue);
    blue_ncc=blue;
    green_ncc = circshift(green,offset_bg);
    red_ncc = circshift(red,offset_br);
    rgb_ncc(:,:,1) = red_ncc;
    rgb_ncc(:,:,2) = green_ncc;
    rgb_ncc(:,:,3) = blue_ncc;
    imwrite(rgb_ncc,strcat("image",int2str(i),'-ncc.jpg'),"jpg");
    fprintf("image%d: ncc align offset between blue and green: [%d,%d] \n",i,offset_bg(1),offset_bg(2));
    fprintf("image%d: ncc align offset between blue and red: [%d,%d] \n",i,offset_br(1),offset_br(2));
    clear rgb_ncc

    [offset_bg,offset_br]=im_align3(red,green,blue);
    %[red_ssd,green_ssd,blue_ssd] = im_align1(red,green,blue);
    blue_corner=blue;
    green_corner = circshift(green,offset_bg);
    red_corner = circshift(red,offset_br);
    rgb_corner(:,:,1) = red_corner;
    rgb_corner(:,:,2) = green_corner;
    rgb_corner(:,:,3) = blue_corner;
    imwrite(rgb_corner,strcat("image",int2str(i),'-corner.jpg'),"jpg");
    fprintf("image%d: harris detection and mapping offset between blue and green: [%d,%d] \n",i,offset_bg(1),offset_bg(2));
    fprintf("image%d: harris detection and mapping offset between blue and red: [%d,%d] \n",i,offset_br(1),offset_br(2));
    clear rgb_corner


end