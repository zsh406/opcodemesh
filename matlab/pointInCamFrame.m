function resBool=pointInCamFrame(ext, cameraInteriors, point_3D)
    nPoints=size(point_3D,1);
    resBool=zeros(nPoints,1);
    Xs=ext(1);Ys=ext(2);Zs=ext(3);w2=ext(4);p2=ext(5);k2=ext(6);
    f=cameraInteriors.f;
    cw=cameraInteriors.cw;
    ch=cameraInteriors.ch;
    Rw=m_rotx(w2);
    Rp=m_roty(p2);
    Rk=m_rotz(k2);
    R=Rk*Rp*Rw;
    m=R;   

    m11=  m(1,1);
    m12=  m(1,2);
    m13=  m(1,3);
    m21=  m(2,1);
    m22=  m(2,2);
    m23=  m(2,3);
    m31=  m(3,1);
    m32=  m(3,2);
    m33=  m(3,3);
    for i=1:nPoints
        X=point_3D(i,1);Y=point_3D(i,2);Z=point_3D(i,3);
        X_=m11*(X-Xs)+m21*(Y-Ys)+m31*(Z-Zs);
        Y_=m12*(X-Xs)+m22*(Y-Ys)+m32*(Z-Zs);
        Z_=m13*(X-Xs)+m23*(Y-Ys)+m33*(Z-Zs);
        x=-f*X_/Z_;
        y=-f*Y_/Z_;
        if abs(x)<cw/2 && abs(y)<ch/2
            resBool(i)=1;
        end   
    end
    resBool=logical(resBool);
end

function rotmat=m_rotx(alpha)
   rotmat = [1 0 0;0 cosd(alpha) -sind(alpha); 0 sind(alpha) cosd(alpha)];   
end

function rotmat=m_roty(beta)
   rotmat = [cosd(beta) 0 sind(beta); 0 1 0; -sind(beta) 0 cosd(beta)];   
end

function rotmat=m_rotz(gamma)
   rotmat = [cosd(gamma) -sind(gamma) 0; sind(gamma) cosd(gamma) 0; 0 0 1];   
end