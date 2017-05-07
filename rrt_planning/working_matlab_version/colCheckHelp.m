function collideCheck = colCheckHelp(g1, e1, g2, e2)
%COLLISION_CHECKER Summary of this function goes here
%   Detailed explanation goes here
collideCheck = 1;

Pa = g1(1:3,4);
Ax = g1(1:3,1);
Ay = g1(1:3,2);
Az = g1(1:3,3);
Wa = e1(1);
Ha = e1(2);
Da = e1(3);

Pb = g2(1:3,4);
Bx = g2(1:3,1);
By = g2(1:3,2);
Bz = g2(1:3,3);
Wb = e2(1);
Hb = e2(2);
Db = e2(3);

T = Pb - Pa;

Rxx = dot(Ax, Bx);
Rxy = dot(Ax, By);
Rxz = dot(Ax, Bz);
Ryy = dot(Ay, By);
Ryz = dot(Ay, Bz);
Rzz = dot(Az, Bz);

%Case 1:
if abs(dot(T, Ax)) > Wa + abs(Wb*Rxx) + abs(Hb*Rxy) + abs(Db*Rxz);
    collideCheck = 0;
%     disp('Case 1 no collide')
    return
end

%Case 2:
if abs(dot(T, Ay)) > Ha + abs(Wb*Rxy) + abs(Hb*Ryy) + abs(Db*Ryz);
    collideCheck = 0;
%     disp('Case 2 no collide')
    return
end

%Case 3:
if abs(dot(T,Az)) > Da + abs(Wb*Rxz) + abs(Hb*Ryz) + abs(Db*Rzz);
    collideCheck = 0;
%     disp('Case 3 no collide')
    return
end

%Case 4:
if abs(dot(T,Bx)) > abs(Wa*Rxx) + abs(Ha*Rxy) + abs(Da*Rxz) + Wb;
    collideCheck = 0;
%     disp('Case 4 no collide')
    return
end

%Case 5:
if abs(dot(T,By)) > abs(Wa*Rxy) + abs(Ha*Ryy) + abs(Da*Ryz) + Hb;
    collideCheck = 0;
%     disp('Case 5 no collide')
    return
end

%Case 6:
if abs(dot(T,Bz)) > abs(Wa*Rxz) + abs(Ha*Ryz) + abs(Da*Rzz) + Db;
    collideCheck = 0;
%     disp('Case 6 no collide')
    return
end

%Case 7:
if abs(dot(T,Az)*Rxy - dot(T,Ay)*Rxz) > abs(Ha*Rxz)+abs(Da*Rxy)+abs(Hb*Rxz)+abs(Db*Rxy);
    collideCheck = 0;
%     disp('Case 7 no collide')
    return
end

%Case 8:
if abs(dot(T,Az)*Ryy-dot(T,Ay)*Ryz) > abs(Ha*Ryz)+abs(Da*Ryy)+abs(Wb*Rxz)+abs(Db*Rxx);
    collideCheck = 0;
%     disp('Case 8 no collide')
    return
end

%Case 9:
if abs(dot(T,Az)*Ryz-dot(T,Ay)*Rzz) > abs(Ha*Rzz)+abs(Da*Ryz)+abs(Wb*Rxy)+abs(Hb*Rxx);
    collideCheck = 0;
%     disp('Case 9 no collide')
    return
end

%Case 10:
if abs(dot(T,Ax)*Rxz-dot(T,Az)*Rxx) > abs(Wa*Rxz)+abs(Da*Rxx)+abs(Hb*Ryz)+abs(Db*Ryy);
    collideCheck = 0;
%     disp('Case 10 no collide')
    return
end

%Case 11:
if abs(dot(T,Ax)*Ryz-dot(T,Az)*Rxy) > abs(Wa*Ryz)+abs(Da*Rxy)+abs(Wb*Ryz)+abs(Db*Rxy);
    collideCheck = 0;
%     disp('Case 11 no collide')
    return
end

%Case 12:
if abs(dot(T,Ax)*Rzz-dot(T,Az)*Rxz) > abs(Wa*Rzz)+abs(Da*Rxz)+abs(Wb*Ryy)+abs(Hb*Rxy);
    collideCheck = 0;
%     disp('Case 12 no collide')
    return
end

%Case 13:
if abs(dot(T,Ay)*Rxx-dot(T,Ax)*Rxy) > abs(Wa*Rxy)+abs(Ha*Rxx)+abs(Hb*Rzz)+abs(Db*Ryz);
    collideCheck = 0;
%     disp('Case 13 no collide')
    return
end

%Case 14:
if abs(dot(T,Ay)*Rxy-dot(T,Ax)*Ryy) > abs(Wa*Ryy)+abs(Ha*Rxy)+abs(Wb*Rzz)+abs(Db*Rxz);
    collideCheck = 0;
%     disp('Case 14 no collide')
    return
end

%Case 15:
if abs(dot(T,Ay)*Rxz-dot(T,Ax)*Ryz) > abs(Wa*Ryz)+abs(Ha*Rxz)+abs(Wb*Ryz)+abs(Hb*Rxz);
    collideCheck = 0;
%     disp('Case 15 no collide')
    return
end

% disp('Collided with obstacle')
    
end

