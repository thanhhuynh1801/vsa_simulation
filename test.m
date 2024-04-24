
% modelname= 'UR10_2dof.slx';
% % nhập mô hình robot vào:
% open_system(modelname);
% [robot,importInfo] = importrobot(gcs);
% robot.DataFormat = 'row';
% disp(robot.Bodies(1));
% show(robot,'visuals','on','collision','off');
modelname= 'UR10_2dof';
% nhập mô hình robot vào:
% open_system(modelname);
% [robot,importInfo] = importrobot(gcs);
% robot.DataFormat = 'row';
% disp(importInfo);

set_param(modelname,'BlockReduction','off');
set_param(modelname,'StopTime','inf');
set_param(modelname,'simulationMode','normal');
set_param(modelname,'StartFcn','1');
set_param(modelname,'simulationCommand','start');


ur10= createUR10Robot();
T=forward(0,pi/4,ur10);
disp(T)
J=invert(T.t(1),T.t(2),T.t(3),ur10);
disp(J)

set_param([modelname '/Slider Gain'], 'Gain',num2str(J(1)) );
set_param([modelname '/Slider Gain1'], 'Gain',num2str(J(2)));

function ur10= createUR10Robot()
    %bang DH   
    a = [0 , 0.647 ];
    alpha = [pi/2, 0];
    d = [0.1632, 0.197];
    theta = [0, pi/2];

    ur10 = SerialLink([
    Revolute('d', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', theta(1)), ...
    Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', theta(2)), ...
    ]);
    ur10.name = 'abb';
end

%%
function J= invert(px,py,pz,ur10)
roll = deg2rad(0);
pitch = deg2rad(90);
yaw = deg2rad(0);

R11 = cos(pitch) * cos(yaw);
R12 = cos(yaw) * sin(roll) * sin(pitch) - cos(roll) * sin(yaw);
R13 = cos(roll) * cos(yaw) * sin(pitch) + sin(roll) * sin(yaw);
R21 = cos(pitch) * sin(yaw);
R22 = cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw);
R23 = -cos(yaw) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw);
R31 = -sin(pitch);
R32 = cos(pitch) * sin(roll);
R33 = cos(roll) * cos(pitch);
 T = [R11, R12, R13, px;
     R21, R22, R23, py;
     R31, R32, R33, pz;
     0, 0, 0, 1];
 J = ur10.ikine(T, [0, 0, 0], 'mask', [1, 1, 0, 0, 0, 0]) ;
end
%%
function T= forward(theta1, theta2,ur10)
 theta = [theta1, theta2];
 T = ur10.fkine(theta);
end
