% Simscape(TM) Multibody(TM) version: 7.5

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(10).translation = [0.0 0.0 0.0];
smiData.RigidTransform(10).angle = 0.0;
smiData.RigidTransform(10).axis = [0.0 0.0 0.0];
smiData.RigidTransform(10).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 117 105];  % mm
smiData.RigidTransform(1).angle = 3.1415926535897882;  % rad
smiData.RigidTransform(1).axis = [-2.468850131082261e-15 0.70710678118654879 0.70710678118654624];
smiData.RigidTransform(1).ID = "UCS[UR10_movable:Coordinate System1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [0 230.00000000000006 104.59999999999999];  % mm
smiData.RigidTransform(2).angle = 3.1415926535897865;  % rad
smiData.RigidTransform(2).axis = [-0 -1 -0];
smiData.RigidTransform(2).ID = "UCS[UR10_movable:Coordinate System4]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [44.999999999999929 792 -69.000000000000014];  % mm
smiData.RigidTransform(3).angle = 3.1415926535897882;  % rad
smiData.RigidTransform(3).axis = [-0.70710678118654879 -0.70710678118654624 2.468850131082261e-15];
smiData.RigidTransform(3).ID = "UCS[UR10_movable:Coordinate System5]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-1.9428902930940239e-13 214.00000000000028 -175.99999999999076];  % mm
smiData.RigidTransform(4).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(4).axis = [-0.57735026918962562 -0.57735026918962606 -0.57735026918962551];
smiData.RigidTransform(4).ID = "B[Motor2-1:-:Link1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [-3.780087354243733e-12 213.80000000000354 -176.00000000000318];  % mm
smiData.RigidTransform(5).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(5).axis = [-0.57735026918962584 -0.57735026918962606 -0.57735026918962551];
smiData.RigidTransform(5).ID = "F[Motor2-1:-:Link1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [0 0 0];  % mm
smiData.RigidTransform(6).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(6).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(6).ID = "B[Base-1:-:Motor1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-1.9041327017064116e-14 -1.4210854715202004e-13 -1.0075046864250228e-12];  % mm
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(7).ID = "F[Base-1:-:Motor1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [0 127.99999999998934 -86.000000000000469];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [1 0 0];
smiData.RigidTransform(8).ID = "B[Motor1-1:-:Motor2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [-1.0516032489249483e-12 128.00000000000085 -86.000000000001648];  % mm
smiData.RigidTransform(9).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(9).axis = [1 1.0396454646672375e-32 2.1866321785694499e-16];
smiData.RigidTransform(9).ID = "F[Motor1-1:-:Motor2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [2.4074515813577029 117.97596146156042 104.35383910409857];  % mm
smiData.RigidTransform(10).angle = 0;  % rad
smiData.RigidTransform(10).axis = [0 0 0];
smiData.RigidTransform(10).ID = "RootGround[Base-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(4).mass = 0.0;
smiData.Solid(4).CoM = [0.0 0.0 0.0];
smiData.Solid(4).MoI = [0.0 0.0 0.0];
smiData.Solid(4).PoI = [0.0 0.0 0.0];
smiData.Solid(4).color = [0.0 0.0 0.0];
smiData.Solid(4).opacity = 0.0;
smiData.Solid(4).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.70632432004848111;  % kg
smiData.Solid(1).CoM = [0.0069910296491889779 18.435440807891595 0.15935346628633262];  % mm
smiData.Solid(1).MoI = [1155.5761230041232 2128.9701930927931 1145.8454044024281];  % kg*mm^2
smiData.Solid(1).PoI = [-0.19954489044347723 -0.048008422193091982 0.078755298982372227];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Base*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 3.0735712210870503;  % kg
smiData.Solid(2).CoM = [0.0042460870651403046 116.47687966140877 -11.617056742969771];  % mm
smiData.Solid(2).MoI = [11477.104292670321 10119.738354586645 10716.625205831901];  % kg*mm^2
smiData.Solid(2).PoI = [858.53287847317881 0.23956324997467973 -0.99327203259321406];  % kg*mm^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Motor1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 3.0735127938466262;  % kg
smiData.Solid(3).CoM = [-0.0032671592795134835 139.61711748479703 -164.47623111236257];  % mm
smiData.Solid(3).MoI = [11476.951400128746 10715.684304292945 10119.020012679584];  % kg*mm^2
smiData.Solid(3).PoI = [858.66388850635713 -0.7520268525650724 0.3613347988972358];  % kg*mm^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Motor2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 4.305125990827829;  % kg
smiData.Solid(4).CoM = [-3.5301822548955246e-06 440.73812693728001 -175.99999985381257];  % mm
smiData.Solid(4).MoI = [82156.660428584364 6491.5771134858387 82156.66042858432];  % kg*mm^2
smiData.Solid(4).PoI = [-0.00012119581593606891 0 -0.0028556333866653033];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Link1*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(3).Rz.Pos = 0.0;
smiData.RevoluteJoint(3).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = -176.83059151184622;  % deg
smiData.RevoluteJoint(1).ID = "[Motor2-1:-:Link1-1]";

smiData.RevoluteJoint(2).Rz.Pos = -26.586244482548615;  % deg
smiData.RevoluteJoint(2).ID = "[Base-1:-:Motor1-1]";

smiData.RevoluteJoint(3).Rz.Pos = 34.955196186156556;  % deg
smiData.RevoluteJoint(3).ID = "[Motor1-1:-:Motor2-1]";

