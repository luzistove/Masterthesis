%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file execute the joint angle mapping from recorded kinect data to the multibody system in 
% simulation. The method in based on this paper "Real-time tele-operation and tele-walking of 
% humanoid Robot Nao using Kinect Depth Camera". Joint angles are calculated using direction 
% vectors in space with [x y z] coordinates. 
%
%                                   Recorded data in such form:
% [spinemid spinebase hipleft hipright kneeleft kneeright ankleleft ankleright footleft footright]'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all;
% close all;

% Load recorded data 
% for i = 1:20
%     fileName = strcat('ex',num2str(i));
%     load(fileName);
% end

% s(1)=ex1;s(2)=ex2;s(3)=ex3;s(4)=ex4;s(5)=ex5;s(6)=ex6;s(7)=ex7;s(8)=ex8;s(9)=ex9;s(10)=ex10;
% s(1)=ex11;s(2)=ex12;s(3)=ex13;s(4)=ex14;s(5)=ex15;s(6)=ex16;s(7)=ex17;s(8)=ex18;s(9)=ex19;
% s(10)=ex20;
% load('ex10times');
% t = linspace(0,5,50);
% for i = 1:10
    % Calculating direction vectors
%     spineMid2BaseX(i,:) = s(i).x(2,:)-s(i).x(1,:);
%     spineMid2BaseY(i,:) = s(i).y(2,:)-s(i).y(1,:);
%     spineMid2BaseZ(i,:) = s(i).z(2,:)-s(i).z(1,:);
%     
%     
%     hip2KneeLX(i,:) = s(i).x(5,:)-s(i).x(3,:);
%     hip2KneeLY(i,:) = s(i).y(5,:)-s(i).y(3,:);
%     hip2KneeLZ(i,:) = s(i).z(5,:)-s(i).z(3,:);
%     
%     
%     hip2KneeRX(i,:) = s(i).x(6,:)-s(i).x(4,:);
%     hip2KneeRY(i,:) = s(i).y(6,:)-s(i).y(4,:);
%     hip2KneeRZ(i,:) = s(i).z(6,:)-s(i).z(4,:);
%     
%     
%     knee2AnkleLX(i,:) = s(i).x(7,:)-s(i).x(5,:);
%     knee2AnkleLY(i,:) = s(i).y(7,:)-s(i).y(5,:);
%     knee2AnkleLZ(i,:) = s(i).z(7,:)-s(i).z(5,:);
%     
%     
%     knee2AnkleRX(i,:) = s(i).x(8,:)-s(i).x(6,:);
%     knee2AnkleRY(i,:) = s(i).y(8,:)-s(i).y(6,:);
%     knee2AnkleRZ(i,:) = s(i).z(8,:)-s(i).z(6,:);
%     
%     
%     ankle2FootLX(i,:) = s(i).x(9,:)-s(i).x(7,:);
%     ankle2FootLY(i,:) = s(i).y(9,:)-s(i).y(7,:);
%     ankle2FootLZ(i,:) = s(i).z(9,:)-s(i).z(7,:);
%     
%     
%     ankle2FootRX(i,:) = s(i).x(10,:)-s(i).x(8,:);
%     ankle2FootRY(i,:) = s(i).y(10,:)-s(i).y(8,:);
%     ankle2FootRZ(i,:) = s(i).z(10,:)-s(i).z(8,:);
%     
%     
%     % Calculating angles
%     for j = i:length(spineMid2BaseX)
%         % Calculating the pitching angles, hip pitch, knee bend and ankle pitch
%         spineMid2Base = [spineMid2BaseZ(i,:);zeros(1,length(spineMid2BaseX));spineMid2BaseY(i,:)];
%         hip2KneeL = [hip2KneeLZ(i,:);zeros(1,length(hip2KneeLX));hip2KneeLY(i,:)];
%         hip2KneeR = [hip2KneeRZ(i,:);zeros(1,length(hip2KneeRX));hip2KneeRY(i,:)];
%         knee2AnkleL = [knee2AnkleLZ(i,:);zeros(1,length(knee2AnkleLX));knee2AnkleLY(i,:)];
%         knee2AnkleR = [knee2AnkleRZ(i,:);zeros(1,length(knee2AnkleRX));knee2AnkleRY(i,:)];
%         ankle2FootL = [ankle2FootLZ(i,:);zeros(1,length(ankle2FootLX));ankle2FootLY(i,:)];
%         ankle2FootR = [ankle2FootRZ(i,:);zeros(1,length(ankle2FootRX));ankle2FootRY(i,:)];
%         
%         
%         hipPitchL(i,j) = acos(dot(spineMid2Base(:,j),hip2KneeL(:,j))/(norm(spineMid2Base(:,j))*...
%             norm(hip2KneeL(:,j))));
%         hipPitchR(i,j) = acos(dot(spineMid2Base(:,j),hip2KneeR(:,j))/(norm(spineMid2Base(:,j))*...
%             norm(hip2KneeR(:,j))));
%         kneeBendL(i,j) = acos(dot(hip2KneeL(:,j),knee2AnkleL(:,j))/(norm(hip2KneeL(:,j))*...
%             norm(knee2AnkleL(:,j))));
%         kneeBendR(i,j) = acos(dot(hip2KneeR(:,j),knee2AnkleR(:,j))/(norm(hip2KneeR(:,j))*...
%             norm(knee2AnkleR(:,j))));
%         anklePitchL(i,j) = acos(dot(knee2AnkleL(:,j),ankle2FootL(:,j))/(norm(knee2AnkleL(:,j))*...
%             norm(ankle2FootL(:,j))));
%         anklePitchR(i,j) = acos(dot(knee2AnkleR(:,j),ankle2FootR(:,j))/(norm(knee2AnkleR(:,j))*...
%             norm(ankle2FootR(:,j))));
%     end
    
    % Angle decision 
%     for j = 1:length(spineMid2Base)
%         if ((100-spineMid2Base(1,j))-(100-hip2KneeL(1,j))) > 0
%             hipPitchL(i,j) = -hipPitchL(i,j);
%         else
%             hipPitchL(i,j) = hipPitchL(i,j);
%         end
%         
%         if ((100-spineMid2Base(1,j))-(100-hip2KneeR(1,j))) > 0
%             hipPitchR(i,j) = -hipPitchR(i,j);
%         else
%             hipPitchR(i,j) = hipPitchR(i,j);
%         end
%         
%         if ((hip2KneeL(1,j))-(knee2AnkleL(1,j))) > 0
%             kneeBendL(i,j) = -kneeBendL(i,j);
%         else
%             kneeBendL(i,j) = kneeBendL(i,j);
%         end
%         
%         if ((hip2KneeR(1,j))-(knee2AnkleR(1,j))) > 0
%             kneeBendR(i,j) = -kneeBendR(i,j);
%         else
%             kneeBendR(i,j) = kneeBendR(i,j);
%         end
%     end
% end

% load('newdata')
xRecord = inpaintn(xRecord);
yRecord = inpaintn(yRecord);
zRecord = inpaintn(zRecord);


spineMid2BaseX = xRecord(2,:)-xRecord(1,:);
spineMid2BaseY = yRecord(2,:)-yRecord(1,:);
spineMid2BaseZ = zRecord(2,:)-zRecord(1,:);


hip2KneeLX = xRecord(5,:)-xRecord(3,:);
hip2KneeLY = yRecord(5,:)-yRecord(3,:);
hip2KneeLZ = zRecord(5,:)-zRecord(3,:);


hip2KneeRX = xRecord(6,:)-xRecord(4,:);
hip2KneeRY = yRecord(6,:)-yRecord(4,:);
hip2KneeRZ = zRecord(6,:)-zRecord(4,:);


knee2AnkleLX = xRecord(7,:)-xRecord(5,:);
knee2AnkleLY = yRecord(7,:)-yRecord(5,:);
knee2AnkleLZ = zRecord(7,:)-zRecord(5,:);


knee2AnkleRX = xRecord(8,:)-xRecord(6,:);
knee2AnkleRY = yRecord(8,:)-yRecord(6,:);
knee2AnkleRZ = zRecord(8,:)-zRecord(6,:);


ankle2FootLX = xRecord(9,:)-xRecord(7,:);
ankle2FootLY = yRecord(9,:)-yRecord(7,:);
ankle2FootLZ = zRecord(9,:)-zRecord(7,:);


ankle2FootRX = xRecord(10,:)-xRecord(8,:);
ankle2FootRY = yRecord(10,:)-yRecord(8,:);
ankle2FootRZ = zRecord(10,:)-zRecord(8,:);

spineMid2Base = [spineMid2BaseZ;zeros(1,length(spineMid2BaseX));spineMid2BaseY];
hip2KneeL = [hip2KneeLZ;zeros(1,length(hip2KneeLX));hip2KneeLY];
hip2KneeR = [hip2KneeRZ;zeros(1,length(hip2KneeRX));hip2KneeRY];
knee2AnkleL = [knee2AnkleLZ;zeros(1,length(knee2AnkleLX));knee2AnkleLY];
knee2AnkleR = [knee2AnkleRZ;zeros(1,length(knee2AnkleRX));knee2AnkleRY];
ankle2FootL = [ankle2FootLZ;zeros(1,length(ankle2FootLX));ankle2FootLY];
ankle2FootR = [ankle2FootRZ;zeros(1,length(ankle2FootRX));ankle2FootRY];

for j = 1:length(xRecord)
    hipPitchL(:,j) = acos(dot(spineMid2Base(:,j),hip2KneeL(:,j))/(norm(spineMid2Base(:,j))*...
        norm(hip2KneeL(:,j))));
    hipPitchR(:,j) = acos(dot(spineMid2Base(:,j),hip2KneeR(:,j))/(norm(spineMid2Base(:,j))*...
        norm(hip2KneeR(:,j))));
    kneeBendL(:,j) = acos(dot(hip2KneeL(:,j),knee2AnkleL(:,j))/(norm(hip2KneeL(:,j))*...
        norm(knee2AnkleL(:,j))));
    kneeBendR(:,j) = acos(dot(hip2KneeR(:,j),knee2AnkleR(:,j))/(norm(hip2KneeR(:,j))*...
        norm(knee2AnkleR(:,j))));
%     anklePitchL(:,j) = acos(dot(knee2AnkleL(:,j),ankle2FootL(:,j))/(norm(knee2AnkleL(:,j))*...
%         norm(ankle2FootL(:,j))));
%     anklePitchR(:,j) = acos(dot(knee2AnkleR(:,j),ankle2FootR(:,j))/(norm(knee2AnkleR(:,j))*...
%         norm(ankle2FootR(:,j))));  
    
    
    if (spineMid2Base(1,j)-hip2KneeL(1,j)) > 0
        hipPitchL(:,j) = -hipPitchL(:,j);
    else
        hipPitchL(:,j) = hipPitchL(:,j);    
    end

    if (spineMid2Base(1,j)-hip2KneeR(1,j)) > 0
        hipPitchR(:,j) = -hipPitchR(:,j);
    else
        hipPitchR(:,j) = hipPitchR(:,j);
    end
    
    anklePitchL(:,j) = -hipPitchL(:,j)-kneeBendL(:,j);
    anklePitchR(:,j) = -hipPitchR(:,j)-kneeBendR(:,j);
%     if ((hip2KneeL(1,j))-(knee2AnkleL(1,j))) > 0
%         kneeBendL(:,j) = -kneeBendL(:,j);
%     else
%         kneeBendL(:,j) = kneeBendL(:,j);
%     end
% 
%     if ((hip2KneeR(1,j))-(knee2AnkleR(1,j))) > 0
%         kneeBendR(:,j) = -kneeBendR(:,j);
%     else
%         kneeBendR(:,j) = kneeBendR(:,j);
%     end

%     if ((ankle2FootL(1,j))-(knee2AnkleL(1,j))) > 0
%         anklePitchL(:,j) = -anklePitchL(:,j);
%     else
%         anklePitchL(:,j) = anklePitchL(:,j);   
%     end
% 
%     if (ankle2FootR(1,j)-knee2AnkleR(1,j)) > 0
%         anklePitchR(:,j) = -anklePitchR(:,j);
%     else
%         anklePitchR(:,j) = anklePitchR(:,j);   
%     end
end
t = linspace(0,4,length(xRecord));
figure(1)
plot(kneeBendL')
hold on
plot(kneeBendR')
figure(2)
plot(anklePitchL')
hold on
plot(anklePitchR')