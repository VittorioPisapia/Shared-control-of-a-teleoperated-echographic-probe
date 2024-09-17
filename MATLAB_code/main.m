close all
clear all
clc

sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);


if (clientID>-1)
    sim.simxSynchronous(clientID,true)
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
    disp('Connected to remote API server');
    sim.simxSynchronousTrigger(clientID);
    pause(0.1)
    
    %JointHandles
    h=zeros(1,6);
    [r,h(1)]=sim.simxGetObjectHandle(clientID,'Franka_joint1',sim.simx_opmode_blocking);
    [r,h(2)]=sim.simxGetObjectHandle(clientID,'Franka_joint2',sim.simx_opmode_blocking);
    [r,h(3)]=sim.simxGetObjectHandle(clientID,'Franka_joint3',sim.simx_opmode_blocking);
    [r,h(4)]=sim.simxGetObjectHandle(clientID,'Franka_joint4',sim.simx_opmode_blocking);
    [r,h(5)]=sim.simxGetObjectHandle(clientID,'Franka_joint5',sim.simx_opmode_blocking);
    [r,h(6)]=sim.simxGetObjectHandle(clientID,'Franka_joint6',sim.simx_opmode_blocking);
    [r,h(7)]=sim.simxGetObjectHandle(clientID,'Franka_joint7',sim.simx_opmode_blocking);

    global dx dy dz d
    dx = 0;
    dy = 0;
    dz = 0;
    d = 0.05;
    fig = uifigure("Name", "Sliders", "Position", [100, 100, 400, 500]);

    bg = uibuttongroup(fig, "Title","Checks", "Position",[10, 350, 300, 100]);
    check_xy = uiradiobutton(bg, "Text","Plane XY", "Position",[10, 10, 150, 25], "Value", true); 
    check_xz = uiradiobutton(bg, "Text","Plane XZ", "Position",[10, 40, 150, 25]); 
    button_home = uibutton(bg, "Position", [200, 40, 75, 25], "Text", "HOME", "ButtonPushedFcn", @(button_home, event)updateBtn_Home());

    label_d = uilabel(fig, "Position",[400, 210, 100 , 22], "Text", "0.01");
    label_d_name = uilabel(fig, "Position",[300, 320, 100 , 22], "Text", "Slider Magnitude");
    slider_d = uislider(fig, "Position", [300, 100, 200, 3], "Limits", [0.001, 0.2],"Value", 0.01, "ValueChangedFcn",@(slider_d,event)updateLabel(slider_d,label_d), "Orientation", "vertical");

    button_up = uibutton(fig, "Position", [100, 300, 50, 50], "Text", "UP", "ButtonPushedFcn", @(button_up, event)updateBtn_Up(check_xy));
    button_down = uibutton(fig, "Position", [100, 100, 50, 50], "Text", "DOWN", "ButtonPushedFcn", @(button_down, event)updateBtn_Down(check_xy));
    button_left = uibutton(fig, "Position", [50, 200, 50, 50], "Text", "LEFT", "ButtonPushedFcn", @(button_left, event)updateBtn_Left());
    button_right = uibutton(fig, "Position", [150, 200, 50, 50], "Text", "RIGHT", "ButtonPushedFcn", @(button_right, event)updateBtn_Right());

   
    Dq=eye(7)*5;
    Dr=[500,0,0,0,0;0,500,0,0,0;0,0,650,0,0;0,0,0,20,0;0,0,0,0,500];
    K=[250,0,0,0,0;0,250,0,0,0;0,0,75,0,0;0,0,0,10,0;0,0,0,0,500];
    rs=[+0.42425; -0.00701; +0.83639;pi;+0.64828];
    rd=rs;
    dr=[0;0;0;0;0];
    rp=[0;0;0;0;0];
    dt=0.05;
    e=[0,0,0,0];
    errors = [];
    iteration = 0;
    qp=zeros(7);
    dq=zeros(7);
    Jp=zeros(5,7);
    dqp=zeros(7);
    
    % figure; 
    % hold on;
    % yline(0, '--r', 'LineWidth', 1.5);

    for i=1:7
        	[r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    sim.simxSynchronousTrigger(clientID);
    for i=1:7
            [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming); 
    end
    qp=qn;
    J=JacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
    Jp=J;

    for i=1:7
        sim.simxSetJointMaxForce(clientID,h(i),100,sim.simx_opmode_streaming);
    end
    for i=1:7
        sim.simxSetJointForce(clientID,h(i),0,sim.simx_opmode_oneshot);
    end

    while true
        
        d = str2double(label_d.Text);
        rd=rs+[dx;dy;dz;0;0];        
        % iteration = iteration + 1;
        for i=1:7  
             [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_buffer);   
        end
        dq=(qn-qp)/dt;
        p_e = DKnum(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
        ra=TaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7),p_e);
        dr = (ra-rp)/dt;
        
        disp('errore');
        e = rd-ra

        % errors=[errors,e];
        % xlabel('t');
        % ylabel('Error Norm');
        % title('Real-time Error Norm');

        drawnow;

        g=get_GravityVector(qn);
        J=JacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
        dJ=(J-Jp)/dt;
        c=get_CoriolisVector(qn,dq);
        M=get_MassMatrix(qn);
        
        u=M*pinv(J)*(-dJ*transpose(dq))+c+g+transpose(J)*(K*(rd-ra)-Dr*dr)-Dq*transpose(dq);
        rp = ra;
        qp=qn;
        dqp=dq;


        for i=1:7
            if u(i)>0
                sim.simxSetJointTargetVelocity(clientID,h(i),99999,sim.simx_opmode_oneshot);
            else
                sim.simxSetJointTargetVelocity(clientID,h(i),-99999,sim.simx_opmode_oneshot);
            end
            if abs(u(i))>100
                u(i)=100;
            end
            sim.simxSetJointForce(clientID,h(i),abs(u(i)),sim.simx_opmode_oneshot);
        end

    sim.simxSynchronousTrigger(clientID);
    end
end
function updateBtn_Up(check_xy)
    global dz dy d
    if check_xy.Value==1
        dy = dy+d;
    else 
        dz = dz+d;
    end
end

function updateBtn_Down(check_xy)
    global dz dy d
    if check_xy.Value==1
        dy = dy-d;
    else 
        dz = dz-d;
    end
end

function updateBtn_Left()
    global dx d
    dx = dx-d;
end

function updateBtn_Right()
    global dx d
    dx = dx+d;
end    

function updateBtn_Home()
    global dx dy dz
    dx = 0;
    dy = 0;
    dz = 0;
end

function updateLabel(slider, label)
    label.Text = num2str(slider.Value, "%.2f");
end