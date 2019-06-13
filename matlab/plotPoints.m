function []=plotPoints(point_3D_num, pointIndexList, colorString)

    if isempty(pointIndexList)
        location=point_3D_num;
    else
        ind=ismember(point_3D_num(:,4),pointIndexList);
        location=point_3D_num(ind,:);
    end
    scatter3(location(:,1),location(:,2),location(:,3),colorString);
    hold on;
end

