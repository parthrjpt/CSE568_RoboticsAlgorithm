function [offset_rg,offset_br] = im_align1(red,green,blue)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here,
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
    
    offset_rg=[0,0];
    offset_br=[0,0];
    
    min_ssd_g=inf;
    min_ssd_b=inf;
    
    red_window = red;%(shift+1:r_winw-shift, shift+1:r_winh-shift);

    for ww = -15:15
        for wh = -15:15
            green_window = green(shift+1:g_winw-shift, shift+1:g_winh-shift);
            blue_window = blue(shift+1:b_winw-shift, shift+1:b_winh-shift);
            
            ssd_g = sum(sum(red_window-circshift(green_window,[ww,wh])).^2);
            ssd_b = sum(sum(red_window-circshift(blue_window,[ww,wh])).^2);
            if(ssd_g<=min_ssd_g)
                min_ssd_g=ssd_g;
                offset_rg=[ww,wh];
            end
    
            if(ssd_b<min_ssd_b)
                min_ssd_b=ssd_b;
                offset_br=[ww,wh];
            end
        end
    end
end

