clc; clear;

n = 7;    % Robotun eksen sayısı ABB IRB1200 


orjin1X = 0;
orjin1Y = 0;
orjin1Z = 0.3;

orjin2X = 1.2;
orjin2Y = 0;
orjin2Z = 0.3;

%Matrislerin tanımlanması, Hazırlık aşamaları
o1 = zeros(3,1); x1 = zeros(3,n); x1(1,:) = ones(1,n); 
y1 = zeros(3,n); y1(2,:) = ones(1,n); 
z1 = zeros(3,n); z1(3,:) = ones(1,n);
H1 = zeros(6*n,n);
phi1 = eye(6*n);   % n = 6 icin 36 x 36 matris kosegen 1 

%Matrislerin tanımlanması, Hazırlık aşamaları
o2 = zeros(3,1); x2 = zeros(3,n); x2(1,:) = ones(1,n); 
y2 = zeros(3,n); y2(2,:) = ones(1,n); 
z2 = zeros(3,n); z2(3,:) = ones(1,n);
H2 = zeros(6*n,n);
phi2 = eye(6*n);   % n = 6 icin 36 x 36 matris kosegen 1 



% %Eksen hızları belirleme manuel eksen hızı için robot1
% v11 = 0;
% v12 = 0;
% v13 = 0;
% v14 = 0;
% v15 = 0;
% v16 = 0;
% %Eksen hızları belirleme manuel eksen hızı için robot2
% v21 = 0;
% v22 = 0;
% v23 = 0;
% v24 = 0;
% v25 = 0;
% v26 = 0;
% 
% %ROBOTLARIN TİP POİNTLERİ İÇİN HIZ VEKTÖRLERİ:
% R1_wx = 0;
% R1_wy = 0.001;
% R1_wz = 0;
% R1_x = 0;
% R1_y = 0;
% R1_z = 0;
% 
% R2_wx = 0;
% R2_wy = 0.001;
% R2_wz = 0;
% R2_x = 0;
% R2_y = 0;
% R2_z = 0;

%KUTUNUN ORTA NOKTASININ Vc vektörü için:
VC_wx = 0;
VC_wy = 0;
VC_wz = 0;
VC_x = 0.001;
VC_y = 0;
VC_z = 0;



for i=1:n
    phi1 = phi1+diag(ones(6*(n-i),1),-6*i);   %koşegenlerini verme Robot1
end


for i=1:n
    phi2 = phi2+diag(ones(6*(n-i),1),-6*i);   %koşegenlerini verme Robot2
end


color = ('brrggwwg');
axis_max = 0.9;
axis([-axis_max axis_max -axis_max axis_max -axis_max axis_max]);
axis off;
set(gcf, 'color', [0.6 0.6 0.6]);
hold on;



    
for time = 1:200
    cla;
    L1 = [(0.3991/2)*z1(:,1) (0.448)*z1(:,2) (0.042)*y1(:,3) (0.451)*x1(:,4) -(0.082)*z1(:,5) -(0.032)*z1(:,6) (0.032)*x1(:,7)];
    i = 1; H1(6*i + (-5:-3),i) = z1(:,i);
    i = 2; H1(6*i + (-5:-3),i) = y1(:,i);
    i = 3; H1(6*i + (-5:-3),i) = y1(:,i);
    i = 4; H1(6*i + (-5:-3),i) = x1(:,i);
    i = 5; H1(6*i + (-5:-3),i) = y1(:,i);
    i = 6; H1(6*i + (-5:-3),i) = z1(:,i);
    i = 7; H1(6*i + (-5:-3),i) = z1(:,i);
    L2 = [(0.3991/2)*z2(:,1) (0.448)*z2(:,2) (0.042)*y2(:,3) -(0.451)*x2(:,4) -(0.082)*z2(:,5) -(0.032)*z2(:,6) -(0.032)*x2(:,7)];
    i = 1; H2(6*i + (-5:-3),i) = -z2(:,i);
    i = 2; H2(6*i + (-5:-3),i) = -y2(:,i);
    i = 3; H2(6*i + (-5:-3),i) = -y2(:,i);
    i = 4; H2(6*i + (-5:-3),i) = -x2(:,i);
    i = 5; H2(6*i + (-5:-3),i) = -y2(:,i);
    i = 6; H2(6*i + (-5:-3),i) = -z2(:,i);
    i = 7; H2(6*i + (-5:-3),i) = -z1(:,i);

   
    for i=1:n-1 
        phi1(6*i+(4:6),6*i+(-5:-3))=[0 L1(3,i) -L1(2,i); ...
            -L1(3,i) 0 L1(1,i); ...
            L1(2,i) -L1(1,i) 0]; %phi 21 32 43 54 65 icin kosegenler yani sadece L matrisleri
    end

    for i=3:n
        for j=1:i-2
            % phi1(13:18,1:6) = phi1(13:18,7:12)*phi1(7:12,1:6) ornek phi32*phi21
            phi1(6*i+(-5:0),6*j+(-5:0)) = phi1(6*i+(-5:0),6*i+(-11:-6))*phi1(6*i+(-11:-6),6*j+(-5:0));    
        end 
    end
    
    % phi_t icin:
    phi1_t = zeros(6,6*n);
    phi1_t(1:6,(6*n-5):6*n) = [1 0 0 0 0 0;
                        0 1 0 0 0 0;
                        0 0 1 0 0 0;
                        0 -0.032 0 1 0 0;
                        0.032 0 0.149 0 1 0;
                        0 -0.149 0 0 0 1];
    J1 = phi1_t*phi1*H1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

    for i=1:n-1 
        phi2(6*i+(4:6),6*i+(-5:-3))=[0 L2(3,i) -L2(2,i); -L2(3,i) 0 L2(1,i);...
            L2(2,i) -L2(1,i) 0]; %phi 21 32 43 54 65 icin kosegenler icin yani sadece L matrisleri
    end   
    for i=3:n
        for j=1:i-2
            % phi2(13:18,1:6) = phi1(13:18,7:12)*phi1(7:12,1:6) ornek
            phi2(6*i+(-5:0),6*j+(-5:0)) = phi2(6*i+(-5:0),6*i+(-11:-6))*phi2(6*i+(-11:-6),6*j+(-5:0));    
        end 
    end
    
    % phi_t icin:
    phi2_t = zeros(6,6*n);
    phi2_t(1:6,(6*n-5):6*n) = [1 0 0 0 0 0;
                               0 1 0 0 0 0;
                               0        0   1 0 0 0;
                               0     -0.032 0 1 0 0;
                               0.032    0 -0.149 0 1 0;
                               0      0.149 0 0 0 1];
    J2 = phi2_t*phi2*H2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Plot bölümü ROBOT 1
    pR1 = [-0.1 0 0; 0.1 0 0];
    plot3(pR1(:,1),pR1(:,2),pR1(:,3),'b', 'LineWidth', 2);
    p1R1 = o1';
    p2R1 = [orjin1X orjin1Y orjin1Z];    %% orjin koordinatı
    pR1 = [p1R1;p2R1];
    plot3(pR1(:,1),pR1(:,2),pR1(:,3),'b', 'LineWidth', 2);
    p1R1 = p2R1;
    for i = 1:n
        p2R1 = p1R1 + L1(:,i)';
        pR1 = [p1R1;p2R1];
        plot3(pR1(:,1),pR1(:,2),pR1(:,3),'b', 'LineWidth', 6);
        if i<n-1 
            plot3(p2R1(1), p2R1(2), p2R1(3), [color(i) 'o'], 'LineWidth', 4);
        end
        p1R1 = p2R1;
    end
    Gripperyanaluzunluk = 0.03;
    Gripperceneuzunlugu = 0.1;
    p1R1 = p1R1-Gripperyanaluzunluk*y1(:,n)';
    p2R1 = p2R1+Gripperyanaluzunluk*y1(:,n)';
    pR1 = [p1R1;p2R1];
    plot3(pR1(:,1),pR1(:,2),pR1(:,3),'b', 'LineWidth', 3);
    
    p3R1 = p1R1 + Gripperceneuzunlugu*x1(:,n)';
    pR1 = [p1R1;p3R1];
    plot3(pR1(:,1),pR1(:,2),pR1(:,3),'r', 'LineWidth', 3);
    
    p3R1 = p2R1+Gripperceneuzunlugu*x1(:,n)';
    pR1 = [p2R1;p3R1];
    plot3(pR1(:,1),pR1(:,2),pR1(:,3),'r', 'LineWidth', 3);
    drawnow;
    
    thetadotR1 = zeros(n,1); 
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    %Plot bölümü ROBOT 2
    pR2 = [-0.1 0 0; 0.1 0 0];
    plot3(pR2(:,1),pR2(:,2),pR2(:,3),'b', 'LineWidth', 2);
    p1R2 = o2';
    p2R2 = [orjin2X orjin2Y orjin2Z];    %% orjin koordinatı
    pR2 = [p1R2;p2R2];
    plot3(pR2(:,1),pR2(:,2),pR2(:,3),'b', 'LineWidth', 2);
    p1R2 = p2R2;
    for i = 1:n
        p2R2 = p1R2 + L2(:,i)';
        pR2 = [p1R2;p2R2];
        plot3(pR2(:,1),pR2(:,2),pR2(:,3),'b', 'LineWidth', 6);
        if i<n-1 
            plot3(p2R2(1), p2R2(2), p2R2(3), [color(i) 'o'], 'LineWidth', 4);
        end
        p1R2 = p2R2;
    end
    Gripperyanaluzunluk = 0.03;
    Gripperceneuzunlugu = 0.1;
    p1R2 = p1R2-Gripperyanaluzunluk*y2(:,n)';
    p2R2 = p2R2+Gripperyanaluzunluk*y2(:,n)';
    pR2 = [p1R2;p2R2];
    plot3(pR2(:,1),pR2(:,2),pR2(:,3),'b', 'LineWidth', 3);
    
    p3R2 = p1R2 - Gripperceneuzunlugu*x2(:,n)';
    pR2 = [p1R2;p3R2];
    plot3(pR2(:,1),pR2(:,2),pR2(:,3),'r', 'LineWidth', 3);
    
    p3R2 = p2R2-Gripperceneuzunlugu*x2(:,n)';
    pR2 = [p2R2;p3R2];
    plot3(pR2(:,1),pR2(:,2),pR2(:,3),'r', 'LineWidth', 3);
    drawnow;
    
    thetadotR2 = zeros(n,1); 
    
%     phit1c = [1 0 0 0 0 0;   % R1'e göre x yonunde 0.149 m ileride c noktası 
%               0 1 0 0 0 0;
%               0 0 1 0 0 0;
%               0 0 0 1 0 0
%               0 0 0.149 0 1 0
%               0 -0.149 0 0 0 1];     
%     phit2c = [1 0 0 0 0 0;   % R2'e göre -x yonunde 0.149 m ileride c noktası 
%       0 1 0 0 0 0;
%       0 0 1 0 0 0;
%       0 0 0 1 0 0
%       0 0 -0.149 0 1 0
%       0 0.149 0 0 0 1];
    
for i = n:-1:1
    
% belirlenen eksen hızları manuel    
%     thetadotR1(1) = v11;        
%     thetadotR1(2) = v12; 
%     thetadotR1(3) = v13;
%     thetadotR1(4) = v14;
%     thetadotR1(5) = v15;
%     thetadotR1(6) = v16;
%     thetadotR2(1) = v21;     
%     thetadotR2(2) = v22; 
%     thetadotR2(3) = v23;
%     thetadotR2(4) = v24;
%     thetadotR2(5) = v25;
%     thetadotR2(6) = v26;


% % c noktasının hareketi için YENİ ters kinematik çözüm   
     VcR1 = [VC_wx VC_wy VC_wz VC_x VC_y VC_z]';   %büyük V vektoru
     VcR2 = [VC_wx VC_wy VC_wz VC_x VC_y VC_z]';
     thetadotR1 = pinv(J1,1e-2)*VcR1;
     thetadotR2 = pinv(J2,1e-2)*VcR2;
     
% C yani ortak noktanın lineer hızları için ters kinematik çözüm   
%      VcR1 = [VC_wx VC_wy VC_wz VC_x VC_y VC_z]';   %büyük V vektoru
%      VcR2 = [VC_wx VC_wy VC_wz VC_x VC_y VC_z]';
% 
%      thetadotR1 = pinv(J1,1e-2)*phit1c*VcR1;  % ters kinematik çözüm formülü
%      thetadotR2 = pinv(J2,1e-2)*phit2c*VcR2;


    time = H1(6*i+(-5:-3),i);  % sırayla dönme eksenlerini alma
    
    time2 = time;
    time2 = H2(6*i+(-5:-3),i);
    
    %Rodriguez formülü R1 icin
    kdotR1 = [0 -time(3) time(2); ...   %Donme eksenleri skew symmetric
        time(3) 0 -time(1);...
        -time(2) time(1) 0];
    RR1 = eye(3) + sin(thetadotR1(i))*kdotR1 + (1-cos(thetadotR1(i)))*kdotR1^2;
    x1(:,i:n) = RR1*x1(:,i:n);
    y1(:,i:n) = RR1*y1(:,i:n);
    z1(:,i:n) = RR1*z1(:,i:n);
    
    %Rodriguez formülü R2 icin
    kdotR2 = [0 -time2(3) time2(2);...  %Donme eksenleri skew symmetric
        time2(3) 0 -time2(1);...
        -time2(2) time2(1) 0];
    RR2 = eye(3) + sin(thetadotR2(i))*kdotR2 + (1-cos(thetadotR2(i)))*kdotR2^2;
    x2(:,i:n) = RR2*x2(:,i:n);
    y2(:,i:n) = RR2*y2(:,i:n);
    z2(:,i:n) = RR2*z2(:,i:n);
    
end
end



