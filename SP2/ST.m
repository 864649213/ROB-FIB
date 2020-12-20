function error = ST(lndmrk2, asLandM, laserW)
%ST Similarity Transform to calculate the erros
    A = [];
    for i=1:size(asLandM ,2)
        A = [A;[ lndmrk2(asLandM(i),1), lndmrk2(asLandM(i),2),1,0]];
        A = [A;[ lndmrk2(asLandM(i),2),-lndmrk2(asLandM(i),1),0,1]];
    end
    B = [];
    for i=1:size(laserW ,1)
        B = [B; laserW(i,1); laserW(i,2)];
    end
    A;
    B;
    X = inv((A'*A))*A'*B;
    err_x = X(3);
    err_y = X(4);
    err_theta = atan2(X(2),X(1));
    error = [err_x, err_y, err_theta];
end

