function landM = assoLndMrk(lndmrk2,laserW)
%LAN Summary of this function goes here
    [rows, cols] = size(laserW);
    [rows2, cols2] = size(lndmrk2);
    for i = 1:rows
        indexMin = 1;
        minDist = sqrt((laserW(i,1)-lndmrk2(1,1))^2 + (laserW(i,2)-lndmrk2(1,2))^2);
        for j = 2:rows2
            dist = sqrt((laserW(i,1)-lndmrk2(j,1))^2 + (laserW(i,2)-lndmrk2(j,2))^2);
            if dist < minDist
                indexMin = j;
                minDist = dist;
            end 
        end
        landM(i) = indexMin;
    end
end

