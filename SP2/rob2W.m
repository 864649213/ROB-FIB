function laserW = rob2W(laserR,xEst)
%UNTITLED Tranform from robot reference frame to World
    x = xEst(1); y = xEst(2); theta = xEst(3);
    [rows, cols] = size(laserR);
    laserR = [laserR zeros(rows,1) ones(rows,1)]';
    laserT = transl(x,y,0)*trotz(theta)*laserR;
    laserT = laserT';
    laserW = laserT(:,1:2);
end

