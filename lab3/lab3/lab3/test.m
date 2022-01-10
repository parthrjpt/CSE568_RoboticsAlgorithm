function test(red,blue,green)
%TEST Summary of this function goes here
%   Detailed explanation goes here
    kp_r=harris(red);
    kp_b=harris(blue);
    kp_g=harris(green);

    kp_rc=corner(red);
    kp_bc=corner(blue);
    kp_gc=corner(green);


    figure;
    imshow(red);
    axis on
    hold on;
    pts = [kp_r(:,1),kp_r(:,2)];
    plot(pts,'+r');

    figure;
    imshow(red);
    axis on
    hold on;
    pts = [kp_rc(:,1),kp_rc(:,2)];
    plot(pts,'+r');


    figure;
    imshow(blue);
    axis on
    hold on;
    pts = [kp_b(:,1),kp_b(:,2)];
    plot(pts,'+r');

    figure;
    imshow(blue);
    axis on
    hold on;
    pts = [kp_bc(:,1),kp_bc(:,2)];
    plot(pts,'+r');

    
    figure;
    imshow(green);
    axis on
    hold on;
    pts = [kp_g(:,1),kp_g(:,2)];
    plot(pts,'+r');

    figure;
    imshow(green);
    axis on
    hold on;
    pts = [kp_gc(:,1),kp_gc(:,2)];
    plot(pts,'+r');


end

