function shp = create_3d_shape(xmin, xmax, ymin, ymax, zmin, zmax)
    [xg,yg,zg] = meshgrid(xmin:(xmax-xmin)/10:xmax,...
                       ymin:(ymax-ymin)/10:ymax,...
                       zmin:(zmax-zmin)/10:zmax);
    xg = xg(:);
    yg = yg(:);
    zg = zg(:);
    shp = [xg yg zg];
%     figure
%     plot(alphaShape(xg,yg,zg));
end