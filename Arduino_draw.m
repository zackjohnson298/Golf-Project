function Arduino_draw(w)
    phi = deg2rad(w(1));
    theta = deg2rad(w(2));
    psi = deg2rad(w(3));
   
    clf(gcf);

    Floor_v = [-10  10 0;...
                10  10 0;...
               -10 -10 0;...
                10 -10 0];
    Floor_f = [1 2 4 3];

%     patch('Faces',Floor_f,'Vertices',Floor_v,'EdgeColor','None','FaceColor',[0 0 .8],'FaceAlpha',.5);
    hold on
    set(gcf, 'Position', [50, 50, 950, 900])

%     [Base,  Base_f]    = stlread('STL/Base.stl');
%     [Link1, Link1_f]   = stlread('STL/Link1.stl');
%     [Link2, Link2_f]   = stlread('STL/Link2.stl');
%     [Link3, Link3_f]   = stlread('STL/Link3.stl');
%     [Link4, Link4_f]   = stlread('STL/Link4.stl');
%     [Link5, Link5_f]   = stlread('STL/Link5.stl');
%     [Link6, Link6_f]   = stlread('STL/Link6.stl');
%    
    [Board, Board_f]   = stlread('Object.stl');
    
    %Relative Positions:
    rBfromI = [0;       0;       0];
%     r1fromB = [0;       0;      LB]; % base is 8 cm tall
%     r2from1 = [0;       0;      L1]; % link 1 is 14.08 tall and displaced by 1.54 cm in the neg. y-direction
%     r3from2 = [0;      L2;       0]; % link 2 is 25 cm long
%     r4from3 = [0;      L3;       0]; % link 3 is 20 cm long
%     r5from4 = [0;      L4;       0]; % distance from 1st wrist coor. frame to 2nd is 8 cm
%     r6from5 = [0;       0;       0]; % no distance between 2nd and 3rd
%     r7from6 = [0;      LE;       0]; % dist. from 3rd wrist coor. frame to the end effector is 5.25 cm

    %Orientations wrt I:     
    
    
    %Positions wrt I:
    rB = rBfromI;
%     r1 = rB + r1fromB;
%     r2 = r1 + T1*r2from1;
%     r3 = r2 + T2*r3from2;
%     r4 = r3 + T3*r4from3;
%     r5 = r4 + T4*r5from4;
%     r6 = r5 + T5*r6from5;

    %Transform the stl coordinates based upon FK
%     Base_v   = repmat(rB,1,length(Base)) + Base';
%     Link1_v  = repmat(r1,1,length(Link1)) + T1*Link1';
%     Link2_v  = repmat(r2,1,length(Link2)) + T2*Link2';
%     Link3_v  = repmat(r3,1,length(Link3)) + T3*Link3';
%     Link4_v  = repmat(r4,1,length(Link4)) + T4*Link4';
%     Link5_v  = repmat(r5,1,length(Link5)) + T5*Link5';
%     Link6_v  = repmat(r6,1,length(Link6)) + T6*Link6';
%     
%     
    T = rotx(phi)*roty(theta)*rotz(psi);
    Board_v = repmat(rB,1,length(Board)) + T*Board';
    patch('Faces',Board_f, 'Vertices',Board_v', 'EdgeColor','None','FaceColor',[0 0.082353 1]);
    
%     
%     patch('Faces',Base_f, 'Vertices',Base_v', 'EdgeColor','None','FaceColor',[0 0.082353 1]);
%     patch('Faces',Link1_f,'Vertices',Link1_v','EdgeColor','None','FaceColor',[0.901961 0.756863 0.035294]);
%     patch('Faces',Link2_f,'Vertices',Link2_v','EdgeColor','None','FaceColor',[0 0.082353 1]);
%     patch('Faces',Link3_f,'Vertices',Link3_v','EdgeColor','None','FaceColor',[0.901961 0.756863 0.035294]);
%     patch('Faces',Link4_f,'Vertices',Link4_v','EdgeColor','None','FaceColor',[0 0.082353 1]);
%     patch('Faces',Link5_f,'Vertices',Link5_v','EdgeColor','None','FaceColor',[0.901961 0.756863 0.035294]);
%     patch('Faces',Link6_f,'Vertices',Link6_v','EdgeColor','None','FaceColor',[0 0.082353 1]);
%     
    axis equal
    camlight left
    set(gca,'projection','perspective')
    view(3)
    axis([-.2 .2 -.2 .2 -.2 .2])
    hold off
end