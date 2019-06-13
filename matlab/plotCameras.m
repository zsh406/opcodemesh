function p=plotCameras(EO, globalShift, colorString)
showNumbers = false;
nCamera=size(EO,1);
if colorString == "Grey"
    colorString = [.7 .7 .7];
end
    for i=1:nCamera
        EO_c=EO(i,:);
        loc=EO_c(1:3)+globalShift;    

        Rw=rotx(EO_c(4));
        Rp=roty(EO_c(5));
        Rk=rotz(EO_c(6));

        R=Rk*Rp*Rw*rotx(180);
        R=R';
        eulZYX = rad2deg(rotm2eul(R));

        cam=plotCamera('Location',loc,'Orientation',R,'Opacity',0,'Color',colorString);
        if (~isequal(colorString, [.7 .7 .7]) && showNumbers)
           t=text(loc(:,1), loc(:,2) ,loc(:,3),num2str(i)); % camera name
           t.Color='blue';
           t.FontSize=15;
        end

    %      cameras=findall(gcf,'Label','Cam');
    end
end

