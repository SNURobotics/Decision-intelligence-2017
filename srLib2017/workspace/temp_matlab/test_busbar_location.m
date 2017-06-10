%% workspace object locations
numB = 4;
R = zeros(3,3,numB);
p = zeros(3,numB);
robotnum = 2;
Trobot12robot2 = reshape([-1	-1.22465e-16	-1.49976e-32	1.22465e-16	-1	-2.44929e-16	1.49976e-32	-2.44929e-16	1	-1.44	-8.93639e-17	0], 3,4);
Trobot12robot2 = [Trobot12robot2; 0,0,0,1];
if (robotnum == 1)
    % robot1 busbar
    R(:,:,1) = projectToRotZ([0.857752, -0.513389, 0.026321;
        0.512838, 0.858118, 0.025108;
        -0.035476, -0.008038, 0.999338]);
    p(:,1) = [-0.280178, -0.109590, 0.843439]';
    
    R(:,:,2) = projectToRotZ([0.353052, -0.934938, 0.035280;
        0.935125, 0.353826, 0.018640;
        -0.029910, 0.026410, 0.999204]);
    p(:,2) = [-0.266852, -0.020595, 0.843658]';
    
    R(:,:,3) = projectToRotZ([-0.743844, -0.668199, 0.014351;
        0.668328, -0.743831, 0.007326;
        0.005779, 0.015040, 0.999870]);
    p(:,3) = [-0.373395, 0.011193, 0.843244]';
    
    R(:,:,4) = projectToRotZ([0.978449, -0.206485, -0.001310;
        0.206487, 0.978396, 0.010234;
        -0.000831, -0.010284, 0.999947]);
    p(:,4) = [-0.367668, 0.117539, 0.842430]';
else
    % robot2 busbar
    R(:,:,1) = projectToRotZ([0.867343, -0.497550, 0.012696;
        0.495082, 0.865098, 0.080611;
        -0.051091, -0.063632, 0.996665]);
    p(:,1)=[-0.209202, -0.170820, 1.032720]';
    
    R(:,:,2)=projectToRotZ([0.991015, 0.117732, 0.063472;
        -0.117271, 0.993040, -0.010939;
        -0.064318, 0.003398, 0.997924]);
    p(:,2)=[-0.235986, -0.028933, 1.031661]';
    
    R(:,:,3)=projectToRotZ([0.5371, 0.842539, 0.040631;
        -0.843192, 0.537610, -0.001943;
        -0.023481, -0.033216, 0.999172]);
    p(:,3)=[-0.158513, 0.060334, 1.031817]';
    
    R(:,:,4)=projectToRotZ([0.999041, 0.014673, 0.041242;
        -0.014720, 0.999891, 0.000835;
        -0.041225, -0.001442, 0.999149]);
    p(:,4)=[-0.148944, -0.095115, 1.032335]';
    
    
    
end

% ctCase (from robot2)
TctCase2gripper = [RotX(pi), [0.006, 0.031625, 0.01]'; 0,0,0,1];
Rct = projectToRotZ([0.837602, -0.545844, 0.021822;
    0.546203, 0.834787, -0.016668;
    -0.009178, 0.025880, 0.999623]);
pct = [-0.575721, -0.478461, 0.839666]';
Tct = [Rct, pct; 0,0,0,1] * TctCase2gripper^-1;
Tct_robot1 = Trobot12robot2*Tct;
temp = reshape(Tct_robot1(1:3,:), 1,12);
tempchar = [];
for j = 1:12
    tempchar = [tempchar, num2str(temp(j)), ','];
end
display(tempchar)
% jig position
pjig = [-0.614563, -0.033770, 0.748277]';


%%
Tbusbar2gripper_tight = [RotZ(pi/2)*RotX(pi), [0;0;0.015]; 0, 0, 0, 1];


Tbusbar = zeros(4,4,numB);

z_conv = 1.03555 + 0.5*0.1511;
z_plane = 0.910 + 0.003;
z_robot1 = 1.972;
for i = 1:numB
    if robotnum == 1
        Tbusbar(:,:,i) = [R(:,:,i), p(:,i); 0, 0, 0, 1] * Tbusbar2gripper_tight^-1;
    else
        Tbusbar(:,:,i) = Trobot12robot2*[R(:,:,i), p(:,i); 0, 0, 0, 1] * Tbusbar2gripper_tight^-1;
    end
    if robotnum == 1
        if (Tbusbar(3,4,i) > z_robot1 - z_conv - 0.001)
            Tbusbar(3,4,i) = z_robot1 - z_conv - 0.001;
        end
    else
        if (Tbusbar(3,4,i) > z_robot1 - z_plane)
            Tbusbar(3,4,i) = z_robot1 - z_plane;
        end
    end
    temp = reshape(Tbusbar(1:3,:,i), 1,12);
    tempchar = [];
    for j = 1:12
        tempchar = [tempchar, num2str(temp(j)), ','];
    end
    display(tempchar)
end

pjig1 = Trobot12robot2*[pjig; 1];


