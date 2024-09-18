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
    [r,ForceSensor]=sim.simxGetObjectHandle(clientID,'Franka_connection',sim.simx_opmode_blocking);


    global dx dy dz d
    dx = 0;
    dy = 0;
    dz = 0;
    d = 0.05;
    fig = uifigure("Name", "Sliders", "Position", [100, 100, 900, 500]);
    %%% gains =================================================================
    gain = uipanel(fig, "Title","Gains", "Position",[500, 150, 420, 350]);
    label_K_x = uilabel(gain, "Position", [10, 280, 100 , 22], "Text", "Gain over Kx");
    ef_gainK_x = uieditfield(gain,"numeric", "Position", [10, 250, 50, 22], "Limits", [0, 1000], "Value", 250);
    label_K_y = uilabel(gain, "Position",[10, 220, 100 , 22], "Text", "Gain over Ky");
    ef_gainK_y = uieditfield(gain,"numeric", "Position", [10, 190, 50, 22], "Limits", [0, 1000], "Value", 250);
    label_K_z = uilabel(gain, "Position",[10, 160, 100 , 22], "Text", "Gain over Kz");
    ef_gainK_z = uieditfield(gain, "numeric", "Position", [10, 130, 50, 22], "Limits", [0, 1000], "Value", 75;
    label_K_phi = uilabel(gain, "Position",[10, 100, 100 , 22], "Text", "Gain over Kphi");
    ef_gainK_phi = uieditfield(gain, "numeric", "Position", [10, 70, 50, 22], "Limits", [0, 1000], "Value", 50);
    label_K_link4pos = uilabel(gain, "Position",[10, 40, 100 , 22], "Text", "Gain over link 4 position on K");
    ef_gainK_link4pos = uieditfield(gain, "numeric", "Position", [10, 10, 50, 22], "Limits", [0, 1000], "Value", 250);
    
    label_D_x = uilabel(gain, "Position",[180, 280, 100 , 22], "Text", "Gain over Dx");
    ef_gainD_x = uieditfield(gain, "numeric", "Position", [180, 250, 50, 22], "Limits", [0, 1000], "Value", 500);
    label_D_y = uilabel(gain, "Position",[180, 220, 100 , 22], "Text", "Gain over Dy");
    ef_gainD_y = uieditfield(gain, "numeric", "Position", [180, 190, 50, 22], "Limits", [0, 1000], "Value", 500);
    label_D_z = uilabel(gain, "Position",[180, 160, 100 , 22], "Text", "Gain over Dz");
    ef_gainD_z = uieditfield(gain, "numeric", "Position", [180, 130, 50, 22], "Limits", [0, 1000], "Value", 500);
    label_D_phi = uilabel(gain, "Position",[180, 100, 100 , 22], "Text", "Gain over Dphi");
    ef_gainD_phi = uieditfield(gain, "numeric", "Position", [180, 70, 50, 22], "Limits", [0, 1000], "Value", 20);
    label_D_link4pos = uilabel(gain, "Position",[180, 40, 100 , 22], "Text", "Gain over link 4 position on D");
    ef_gainD_link4pos = uieditfield(gain, "numeric", "Position", [180, 10, 50, 22], "Limits", [0, 1000], "Value", 650);
    
    label_Dq = uilabel(gain, "Position",[300, 180, 100 , 22], "Text", "Gain over Dq");
    ef_gainDq = uieditfield(gain, "numeric", "Position", [300, 150, 50, 22], "Limits", [0, 600], "Value", 8);
    
    % label_Aq= uilabel(gain, "Position",[300, 100, 100 , 22], "Text", "Gain over Aq");
    % ef_gainAq= uieditfield(gain, "numeric", "Position", [300, 80, 50, 22], "Limits", [0, 600], "Value", 0);

    label_d = uilabel(fig, "Position",[300, 50, 100 , 22], "Text", "0.01");
    label_d_name = uilabel(fig, "Position",[300, 320, 100 , 22], "Text", "Slider Magnitude");
    slider_d = uislider(fig, "Position", [300, 100, 200, 3], "Limits", [0.001, 0.2],"Value", 0.01, "ValueChangedFcn",@(slider_d,event)updateLabel(slider_d,label_d), "Orientation", "vertical");

    label_phi = uilabel(fig, "Position",[400, 50, 100 , 22], "Text", "3.14");
    label_phi_name = uilabel(fig, "Position",[400, 320, 100 , 22], "Text", "Phi Angle");
    slider_phi = uislider(fig, "Position", [400, 100, 200, 3], "Limits", [pi-deg2rad(20), pi+deg2rad(20)],"Value", pi, "ValueChangedFcn",@(slider_phi,event)updateLabel(slider_phi,label_phi), "Orientation", "vertical");
    
    bg = uibuttongroup(fig, "Title","Checks", "Position",[10, 350, 300, 100]);
    check_xy = uiradiobutton(bg, "Text","Plane XY", "Position",[10, 10, 150, 25], "Value", true); 
    check_xz = uiradiobutton(bg, "Text","Plane XZ", "Position",[10, 40, 150, 25]); 
    button_home = uibutton(bg, "Position", [200, 40, 75, 25], "Text", "HOME", "ButtonPushedFcn", @(button_home, event)updateBtn_Home(slider_phi,label_phi));
    
    button_up = uibutton(fig, "Position", [100, 300, 50, 50], "Text", "UP", "ButtonPushedFcn", @(button_up, event)updateBtn_Up(check_xy));
    button_down = uibutton(fig, "Position", [100, 100, 50, 50], "Text", "DOWN", "ButtonPushedFcn", @(button_down, event)updateBtn_Down(check_xy));
    button_left = uibutton(fig, "Position", [50, 200, 50, 50], "Text", "LEFT", "ButtonPushedFcn", @(button_left, event)updateBtn_Left());
    button_right = uibutton(fig, "Position", [150, 200, 50, 50], "Text", "RIGHT", "ButtonPushedFcn", @(button_right, event)updateBtn_Right());

    %Aq=eye(7)*5;
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

    [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_streaming);

    while true

        d = str2double(label_d.Text);
        rd=rs+[dx;dy;dz;0;0];
        rd(4)=str2double(label_phi.Text);
        if rd(3) <=0
            rd(3) =0;
        end
        % iteration = iteration + 1;
        
        K=[ef_gainK_x.Value,0,0,0,0;
           0,ef_gainK_y.Value,0,0,0;
           0,0,ef_gainK_z.Value,0,0;
           0,0,0,ef_gainK_phi.Value,0;
           0,0,0,0,ef_gainK_link4pos.Value];

        Dr=[ef_gainD_x.Value,0,0,0,0;
           0,ef_gainD_y.Value,0,0,0;
           0,0,ef_gainD_z.Value,0,0;
           0,0,0,ef_gainD_phi.Value,0;
           0,0,0,0,ef_gainD_link4pos.Value];


        Dq=eye(7)*ef_gainDq.Value;

        %Aq=eye(7)*ef_gainAq.Value;

        for i=1:7  
             [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_buffer);   
        end

        [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_buffer);

        dq=(qn-qp)/dt;
        p_e = DKnum(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
        ra=TaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7),p_e);
        dr = (ra-rp)/dt;
        %ddq = (dq-dqp)/dt;
        
        disp('errore');
        e = rd-ra;

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
        %dqp=dq;


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

function updateBtn_Home(slider_phi,label_phi)
    global dx dy dz
    dx = 0;
    dy = 0;
    dz = -0.4;
    slider_phi.Value = pi;
    label_phi.Text = '3.14';
end

function updateLabel(slider, label)
    label.Text = num2str(slider.Value, "%.2f");
end