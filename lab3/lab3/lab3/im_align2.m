function [offset_bg,offset_br] = im_align2(red,green,blue)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here,
    
    %   Detailed explanation goes here,
    [wr,hr]=size(red);
    [wg,hg]=size(green);
    [wb,hb]=size(blue);
    
    shift=30;
    r_winh = hr;
    r_winw = wr;
    g_winh = hg;
    g_winw = wg;
    b_winh = hb;
    b_winw = wb;
    
    offset_bg=[0,0];
    offset_br=[0,0];
    
    max_ncc_g=-inf;
    max_ncc_r=-inf;
    
    blue_window = blue(shift:b_winw-shift, shift:b_winh-shift);
    green_window = green(shift:g_winw-shift, shift:g_winh-shift);
    red_window = red(shift:r_winw-shift, shift:r_winh-shift);
            

    for ww = -15:15
        for wh = -15:15
            shifted_green_win = circshift(green_window,[ww,wh]);
            shifted_red_win = circshift(red_window,[ww,wh]);
            

            mean_r=mean(shifted_red_win(:));
            mean_b=mean(blue_window(:));
            mean_g=mean(shifted_green_win(:));
        
            norm_rwin=double(shifted_red_win)-mean_r;
            norm_bwin=double(blue_window)-mean_b;
            norm_gwin=double(shifted_green_win)-mean_g;
        
            std_red= std((double(shifted_red_win(:))));
            std_blue=std((double(blue_window(:))));
            std_green=std((double(shifted_green_win(:))));
        
            ncc_r_vec = (norm_rwin.*norm_bwin)/(std_red*std_blue);
            ncc_g_vec = (norm_gwin.*norm_bwin)/(std_green*std_blue);

            ncc_r= mean(ncc_r_vec(:));
            ncc_g=  mean(ncc_g_vec(:));


            if(ncc_g>max_ncc_g)
                max_ncc_g=ncc_g;
                offset_bg=[ww,wh];
            end
    
            if(ncc_r>max_ncc_r)
                max_ncc_r=ncc_r;
                offset_br=[ww,wh];
            end
        end
    end
end

