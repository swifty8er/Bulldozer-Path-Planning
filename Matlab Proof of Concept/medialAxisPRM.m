function nodes = medialAxisPRM(CS, Map)
    %load("TestConfigSpace.mat");
%     Map.min_x = 0;
%     Map.max_x = size(CS,2);
%     Map.min_y = 0;
%     Map.max_y = size(CS,1);
%     Map.grid_size = 1;
    CS = imcomplement(CS);
    %convert CS space into medial axis
    MA_Image = bwmorph(CS, 'skel', Inf);
    %{
    figure(8)
    imshow(MA_Image)
    %get node points in CS space
    branch_image = bwmorph(MA_Image, 'branchpoints');
    end_image = bwmorph(MA_Image, 'endpoints');
    node_image = branch_image + end_image;
    %}
    
    [H,theta,rho] = hough(MA_Image);
    peaks = houghpeaks(H,50,'threshold',ceil(0.05*max(H(:))));
    lines = houghlines(MA_Image,theta,rho,peaks,'FillGap',5,'MinLength',7);
    xy = [];
    %figure, imshow(CS), hold on
    for k = 1:length(lines)
        xy = [xy; lines(k).point1; lines(k).point2];
        %{
        plot(xy(:,1),xy(:,2),'g')
        plot(xy(1,1),xy(1,2),'xy')
        plot(xy(2,1),xy(2,2),'xr')
        %}
        %create inbetween points
    end
    
    
    ii = xy(:,2);
    jj = xy(:,1);
    
    %[ii, jj] = find(node_image == 1);
    %[node_x, node_y] = index2Coord(ii, jj, Map);
    nodes(:,1) = (jj-0.5)*Map.grid_size;
    nodes(:,2) = Map.max_y - (ii-0.5)*Map.grid_size;
    new_nodes = [];
    for i = 1:size(nodes,1)-1
        new_nodes = [new_nodes; findMidpoints(nodes(i,:), nodes(i+1,:), 2*Map.disc_radius)];
    end
    nodes = [nodes;new_nodes];
    %remove any outbounds points
    ii = [];
    for i = 1:size(nodes,1)
        [inbounds,onbound] = inpolygon(nodes(i,1), nodes(i,2), Map.boundary(:,1),Map.boundary(:,2));
        if ((inbounds == false) || (onbound == true))
            ii(end+1) = i;
        end
    end
    nodes(ii,:) = [];
    %remove edge end points
    ii = [];
    for i = 1:size(nodes,1)
        j=1;
        too_close = false;
        while ((j < size(Map.boundary,1)-1) && (too_close == false))
            line.Point1_x = Map.boundary(j,1);
            line.Point1_y = Map.boundary(j,2);
            line.Point2_x = Map.boundary(j+1,1);
            line.Point2_y = Map.boundary(j+1,2);
            dist = point2LineDist(line, nodes(i,:));
            if (dist < Map.disc_radius)
                ii(end+1) = i;
                too_close = true;
            end
            j=j+1;
        end
    end
    nodes(ii,:) = [];
    nodes = removeDuplicatePoints(nodes, Map.disc_radius);
    
%     figure(9); hold on;
%     plot(nodes(:,1),nodes(:,2), '*r')
%     plotMap(Map, 9)
    %add random nodes if required may need a proven technique
return;
end


function [x, y] = index2Coord(ii, jj, Map) %implementation needs fixing
    %convert binary image grid indices to xy coordinates
    x = max(Map.min_x, min(Map.max_x, Map.min_x + (ii - 0.5)*Map.grid_size));
    y = max(Map.min_y, min(Map.max_y, Map.min_y + (jj - 0.5)*Map.grid_size));
return;
end

function new_pts = removeDuplicatePoints(pts, tol)
    new_pts = [];
    for i = 1:size(pts,1)-1
        j = i+1;
        in_tolerance = true;
        while ((j <= size(pts,1)) && (in_tolerance == true))
            dist = ptDist(pts(i,:), pts(j,:));
            if (dist < tol)
                in_tolerance = false;
            end
            j = j+1;
        end
        if (in_tolerance == true)
            new_pts = [new_pts; pts(i,:)];
        end
    end

return;
end