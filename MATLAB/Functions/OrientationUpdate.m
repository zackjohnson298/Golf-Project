function [q,wErr] = OrientationUpdate(qLast,Sa,Sm,Sw,B,C,dt,wErrLast)
    
    Sm = Sm/norm(Sm);
    h = quaternProd(qLast,quaternProd(Sm,quaternConj(qLast)));
    h = h/norm(h);
    b = [0 sqrt(h(2)^2 + h(3)^2) 0 h(4)];
    
    [df,dfMag] = Del_f(qLast,Sa,b,Sm);
    qdotErr = df/dfMag;
    wErr = [wErrLast;2*quaternProd(quaternConj(qLast),qdotErr)];
    wb = [0 0 0 0];
    for ii = 1:length(dt)
        wb = wb + C*(wErr(ii,:)*dt(ii));
    end
    wc = Sw - wb;
    qdotw = .5*quaternProd(qLast,wc);
    qdotest = qdotw - B*qdotErr;
    q = qLast + qdotest*dt(end);
    q = q/norm(q);

end


% internal functions
function [df,dfMag] = Del_f(q,a,b,m)

    df = (Jgb(q,b).'*fgb(q,a,b,m)).';
    dfMag = norm(df);

end

function f = fgb(q,a,b,m)

    f = [fg(q,a);fb(q,b,m)];

end

function f = fg(q,a)
    
    a = a/norm(a);
    ax = a(1);
    ay = a(2);
    az = a(3);
    f = [2*(q(2)*q(4) - q(1)*q(3)) - ax;
         2*(q(1)*q(2) + q(3)*q(4)) - ay;
         2*(0.5 - q(2)^2 - q(3)^2) - az];

end

function f = fb(q,b,m)

    m = m/norm(m);
    b = b/norm(b);
    mx = m(2);
    my = m(3);
    mz = m(4);
    bx = b(2);
    bz = b(4);
    
    f = [2*bx*(0.5 - q(3)^2 - q(4)^2) + 2*bz*(q(2)*q(4) - q(1)*q(3)) - mx;
         2*bx*(q(2)*q(3) - q(1)*q(4)) + 2*bz*(q(1)*q(2) + q(3)*q(4)) - my;
         2*bx*(q(1)*q(3) + q(2)*q(4)) + 2*bz*(0.5 - q(2)^2 - q(3)^2) - mz];      

end

function J = Jgb(q,b)

    J = [Jg(q);Jb(q,b)];

end

function J = Jg(q)

    J = [-2*q(3),  2*q(4), -2*q(1), 2*q(2);
          2*q(2),  2*q(1),  2*q(4), 2*q(3);
               0, -4*q(2), -4*q(3),      0];

end

function J = Jb(q,b)

    bx = b(2);
    bz = b(4);
    
    J = [          -2*bz*q(3),           2*bz*q(4), -4*bx*q(3)-2*bz*q(1), -4*bx*q(4)+2*bz*q(2);
         -2*bx*q(4)+2*bz*q(2), 2*bx*q(3)+2*bz*q(1),  2*bx*q(2)+2*bz*q(4), -2*bx*q(1)+2*bz*q(3);
                    2*bx*q(3), 2*bx*q(4)-4*bz*q(2),  2*bx*q(1)-4*bz*q(3),            2*bx*q(2)];

end