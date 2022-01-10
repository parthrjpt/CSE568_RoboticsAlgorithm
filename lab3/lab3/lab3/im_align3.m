function [offset_bg,offset_br] = im_align3(red,green,blue)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    [wr,hr]=size(red);
    [wg,hg]=size(green);
    [wb,hb]=size(blue);
    
    shift=0;
    r_winh = hr;
    r_winw = wr;
    g_winh = hg;
    g_winw = wg;
    b_winh = hb;
    b_winw = wb;
    
    offset_bg=[0,0];
    offset_br=[0,0];
    kp_r = harris(red);
   
    kp_b = harris(blue);
    kp_g = harris(green);
    min_dist_br=inf;
    min_dist_bg=inf;
    for i = 1:300
        b_idx= floor(1 + (size(kp_b,1)-1).*rand(1,1));
        x1=kp_b(b_idx,1);
        y1=kp_b(b_idx,2);
        for j= 1:size(kp_r,1)
            x2=kp_r(j,1);
            y2=kp_r(j,2);
            dist_br=blue(x1,y1)^2-red(x2,y2)^2;
            dist_br=sqrt(double(dist_br));
            if(dist_br<=min_dist_br)
                min_dist_br=dist_br;
                offset_br=[x2-x1,y2-y1];
            end
        end
        for j= 1:size(kp_g,1)
            x3=kp_g(j,1);
            y3=kp_g(j,2);
            dist_bg=blue(x1,y1)^2-green(x3,y3)^2;
            dist_bg=sqrt(double(dist_bg));
            if(dist_bg<=min_dist_bg)
                min_dist_bg=dist_bg;
                offset_bg=[x3-x1,y3-y1];
            end
        end
end

