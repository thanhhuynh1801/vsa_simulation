function varargout = gui_ur10(varargin)
% GUI_UR10 MATLAB code for gui_ur10.fig
%      GUI_UR10, by itself, creates a new GUI_UR10 or raises the existing
%      singleton*.
%
%      H = GUI_UR10 returns the handle to a new GUI_UR10 or the handle to
%      the existing singleton*.
%
%      GUI_UR10('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_UR10.M with the given input arguments.
%
%      GUI_UR10('Property','Value',...) creates a new GUI_UR10 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_ur10_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_ur10_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_ur10

% Last Modified by GUIDE v2.5 23-Apr-2024 15:16:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_ur10_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_ur10_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui_ur10 is made visible.
function gui_ur10_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_ur10 (see VARARGIN)

% Choose default command line output for gui_ur10
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_ur10 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_ur10_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta1_Callback(hObject, eventdata, handles)
% hObject    handle to theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta1 as text
%        str2double(get(hObject,'String')) returns contents of theta1 as a double


% --- Executes during object creation, after setting all properties.
function theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta2_Callback(hObject, eventdata, handles)
% hObject    handle to theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta2 as text
%        str2double(get(hObject,'String')) returns contents of theta2 as a double


% --- Executes during object creation, after setting all properties.
function theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function px_Callback(hObject, eventdata, handles)
% hObject    handle to px (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of px as text
%        str2double(get(hObject,'String')) returns contents of px as a double


% --- Executes during object creation, after setting all properties.
function px_CreateFcn(hObject, eventdata, handles)
% hObject    handle to px (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function py_Callback(hObject, eventdata, handles)
% hObject    handle to py (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of py as text
%        str2double(get(hObject,'String')) returns contents of py as a double


% --- Executes during object creation, after setting all properties.
function py_CreateFcn(hObject, eventdata, handles)
% hObject    handle to py (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pz_Callback(hObject, eventdata, handles)
% hObject    handle to pz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pz as text
%        str2double(get(hObject,'String')) returns contents of pz as a double


% --- Executes during object creation, after setting all properties.
function pz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global modelname;
global ur10;
modelname= 'UR10_2dof';
set_param(modelname,'BlockReduction','off');
set_param(modelname,'StopTime','inf');
set_param(modelname,'simulationMode','normal');
set_param(modelname,'StartFcn','1');
set_param(modelname,'simulationCommand','start');
global theta1_pre;
global theta2_pre;
theta1_pre=0;
theta2_pre=0;

%bang DH   
    a = [0 , 0.647 ];
    alpha = [pi/2, 0];
    d = [0.1632, 0.197];
    theta = [0, pi/2];

    ur10 = SerialLink([
    Revolute('d', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', theta(1)), ...
    Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', theta(2)), ...
    ]);
    T = ur10.fkine([0,0]);
%     J=invert(T.t(1),T.t(2),T.t(3),ur10);
    J = ur10.ikine(T, [0, 0, 0], 'mask', [1, 1, 0, 0, 0, 0]) ;

set(handles.px,'string',num2str(round(T.t(1), 4)));
set(handles.py,'string',num2str(round(T.t(2), 4)));
set(handles.pz,'string',num2str(round(T.t(3), 4)));
set(handles.theta1,'string',num2str(0));
set(handles.theta2,'string',num2str(0));
set_param([modelname '/Slider Gain'], 'Gain',num2str(0));
set_param([modelname '/Slider Gain2'], 'Gain',num2str(0));
    axes(handles.axes1);
     hold on;
%      scatter3(X, Y, Z,1, 'r', 'filled');
     ur10.plot(J);
     xlabel('X');
     ylabel('Y');
     zlabel('Z');
     axis([-1.5 1.5 -1.5 1.5 -1 1.5]);
     grid on;

set(handles.run, 'Enable', 'off');


% --- Executes on button press in move.
function move_Callback(hObject, eventdata, handles)
% hObject    handle to move (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global modelname;
global theta1_pre;
global theta2_pre;
global ur10;

target_th1 = str2double(handles.theta1.String)*pi/180;
target_th2 = str2double(handles.theta2.String)*pi/180;
C1 = (pi/180)*(get(handles.c1,'Value'));  
C2 = (pi/180)*(get(handles.c2,'Value'));
qd1_conver=conver_qd(C1,target_th1,theta1_pre);
qd2_conver=conver_qd(C2,target_th2,theta2_pre);
set_param([modelname '/Slider Gain1'], 'Gain',num2str(qd1_conver));
set_param([modelname '/Slider Gain3'], 'Gain',num2str(qd2_conver));
T = ur10.fkine([target_th1,target_th2]);

currentX = str2double(handles.px.String);        % đọc vị trí của robot hiện tại trên gui
currentY = str2double(handles.py.String);
currentZ = str2double(handles.pz.String);

targetx = T.t(1);          % đọc vị trí cần tới của robot trên gui
targety = T.t(2);
targetz = T.t(3);
% K= invert(targetx,targety,targetz,ur10);           % giải động học thuận 

desiredVector = [targetx - currentX, targety - currentY, targetz - currentZ];    % quy hoạch quỹ đạo tính thời gian di chuyển robot với vận tốc và gia tốc trê gui
normDesiredVector = norm(desiredVector);

qMax = normDesiredVector;  

aMax = 2;
vMax = sqrt(qMax*aMax);
vmax_set= 3;
if (vmax_set< vMax)
    vMax=vmax_set;
end

t1      = vMax/aMax;
tm      = (qMax - aMax*t1^2)/vMax;
tmax    = 2*t1 + tm;
t2      = tmax - t1;

t       = 0:0.05:tmax;
lengthT = length(t);                    % khỏi tạo các biến trống để lưu giá trị (khớp, momen, vị trí)
a       = zeros(lengthT,1);
v       = zeros(lengthT,1);
q       = zeros(lengthT,1);
th1_pre=theta1_pre;
th2_pre=theta2_pre;
Th_1=zeros(lengthT,1);
Th_2=zeros(lengthT,1);

for i = 1:1:lengthT                  % quy hoạch quỹ đạo tam giác cho robot
    if (t(i) < t1)
        a(i) = aMax;
        v(i) = aMax*t(i);
        q(i) = 0.5*aMax*t(i)^2;
    elseif (t(i) < t2)
        a(i) = 0;
        v(i) = vMax;
        q(i) = 0.5*aMax*t1^2 + vMax*(t(i)-t1);
    else
        a(i) = -aMax;
        v(i) = vMax - aMax*(t(i)-t2);
        q(i) = qMax - 0.5*aMax*(tmax-t(i))^2;
    end

    qStd = q(i)/qMax;
    Th_1(i)=th1_pre+(target_th1-th1_pre)*qStd;       % quy hoạch đạo theo joint space
    Th_2(i)=th2_pre+(target_th2-th2_pre)*qStd;
    J=[Th_1(i),Th_2(i)];
    T = ur10.fkine([Th_1(i),Th_2(i)]);

    set(handles.px,'string',num2str(round(T.t(1), 4)));
    set(handles.py,'string',num2str(round(T.t(2), 4)));
    set(handles.pz,'string',num2str(round(T.t(3), 4)));

    set_param([modelname '/Slider Gain'], 'Gain',num2str(Th_1(i)));
    set_param([modelname '/Slider Gain2'], 'Gain',num2str(Th_2(i)));

    axes(handles.axes1);
    hold on;
    ur10.plot(J);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis([-1 1 -1 1 -1 1.5]);
    % Hiển thị lưới
    grid on;
    pause(0.1);
    theta1_pre = Th_1(i);
    theta2_pre = Th_2(i);

end
set_param([modelname '/Integrator3'], 'InitialCondition',num2str(theta1_pre));
set_param([modelname '/Integrator1'], 'InitialCondition',num2str(theta2_pre));

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% evntdata  reserved - to be defined in a future version of MATLAB
% handl
global modelname;
set_param(modelname,'simulationCommand','stop');
set(handles.run, 'Enable', 'on');



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function c1_Callback(hObject, eventdata, handles)
% hObject    handle to c1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function c1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to c1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function c2_Callback(hObject, eventdata, handles)
% hObject    handle to c2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function c2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to c2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double


% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
