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


    global dx dy dz d ForceZ flag_doing_echo phi rd
    dx = 0;
    dy = 0;
    dz = 0;
    d = 0.05;
    ForceZ = 0;
    phi = 0;
    rd = zeros(6,1);

    SAFETY_VALUE = 8;
    flag_can_echo = true;
    flag_doing_echo = false;


    %% INITIALIZATION

    %%%%%%%%%%%
    %Aq=eye(7)*5;
    Dq=eye(7)*5;     
    Dr=eye(6);
    K=eye(6);
    qp=zeros(7);    
    dqp=zeros(7);   
    Jp=zeros(6,7);
    dt=0.05;
    T=4;
    t=transpose(0:dt:T);
    e=[0,0,0,0,0,0];
    dq=zeros(7,1);

    %%%%%%%%%%%
    %start position
    rs=[+0.42425; -0.00701; +0.83639;pi;+0.64828;0];
    rd=rs;

    rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2), ones(length(t),1)*rd(3), ones(length(t),1)*rd(4),ones(length(t),1)*rd(5),ones(length(t),1)*rd(6)];
    drd = [zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1)];
    ddrd = [zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
    
    dr=[0;0;0;0;0;0];
    rp=[0;0;0;0;0;0];

        Km=[250,0,0,0,0,0;
           0,250,0,0,0,0;
           0,0,75,0,0,0;
           0,0,0,45,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0];
        Dm=[300,0,0,0,0,0;
           0,300,0,0,0,0;
           0,0,300,0,0,0;
           0,0,0,20,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0];
        Mm=[50,0,0,0,0,0;
           0,50,0,0,0,0;
           0,0,50,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0];
        Dq=eye(7)*0;

    %=============================
    t = 0;
    time = 1;


    %% FLUSH THE BUFFER

    for i=1:7
        	[r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    sim.simxSynchronousTrigger(clientID);

    %% READ FIRST JOINT POSITION AND SET UP STARTING JOINT TORQUE
    for i=1:7
            [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming); 
    end
    qp=qn;
    J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
    Jp=J;
    rp =EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));

    for i=1:7
        sim.simxSetJointMaxForce(clientID,h(i),50,sim.simx_opmode_streaming);
    end
    for i=1:7
        sim.simxSetJointForce(clientID,h(i),0,sim.simx_opmode_oneshot);
    end

    %% SIGNAL STREAMING
    sim.simxSetFloatSignal(clientID,'error_x',e(1),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'error_y',e(2),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'error_z',e(3),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'error_phi',e(4),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'error_z4',e(5),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'error_phi',e(6),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'rd_x',rd(1),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'rd_y',rd(2),sim.simx_opmode_streaming);
    sim.simxSetFloatSignal(clientID,'rd_z',rd(3),sim.simx_opmode_streaming);

    %% FORCE SENSOR 
    [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_streaming);

    % SIMULATION LOOP

    while t<T

        for i=1:7
            [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
        end

        dq=(qn-qp)/dt;
        disp('t')
        time
        disp('qn')
        qn
        disp('ra')
        ra=EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7)) %task attuale
        disp('rd1')
        rd1(time,:)
        disp('drd')
        drd(time,:)
        disp('ddrd')
        ddrd(time,:)


        dr = (ra-rp)/dt;

        g=get_GravityVector(qn);
        c=get_CoriolisVector(qn,dq);
        M=get_MassMatrix(qn);
        J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
        dJ=(J-Jp)/dt;

        Mr=pinv(J*pinv(M)*transpose(J));

        %% GAINS
        threshold = 0.17;   %PHI control starts at THETA=2.97
        if rd1(time,4)>pi-threshold && rd1(time,4)<pi+threshold
            disp('if')
            Km(6,6)=0;
            Dm(6,6)=0;
        else
            Km(6,6)=1;
            Dm(6,6)=1;
        end
    
        %== possibile forma con check sulla forza di contatto ===========================
        [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_buffer);
        %fz = -force(3);
        
        %control law
        %u=M*pinv(J)*(ddrd(time,:)-dJ*dq+pinv(Mm)*(Dm*(drd(time,:)-dr)+Km*(rd1(time,:)-ra)))+c+g+transpose(J)*(Mr*pinv(Mm)-eye(6)*transpose([force,torque]))-Dq*dq;
        u=M*pinv(J)*(ddrd(time,:)-dJ*transpose(dq))+c+g+transpose(J)*(Km*(rd1(time,:)-ra)+Dm*(drd(time,:)-dr))-Dq*transpose(dq);
        % u=M*pinv(J)*(-dJ*dq)+c+g+transpose(J)*(Km*(rd-ra)+Dm*(-dr))-Dq*dq;
        % u=M*pinv(J)*(-dJ*transpose(dq))+c+g+transpose(J)*(Km*(rd-ra)-Dm*dr)-Dq*transpose(dq)
        % %main


        % while true
        %     if fz < 5
        %         rd(3) = rd(3)-0.01;
        %     else
        %         break;
        %     end
        % end  
        %============================================
        % pause();
        
        qp=qn;
        rp=ra;
        Jp=J;
        t = t+dt;
        time = time+1;

        sim.simxSetFloatSignal(clientID,'error_x',e(1),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'error_y',e(2),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'error_z',e(3),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'error_theta',e(4),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'error_z4',e(5),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'error_phi',e(6),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'rd_x',rd(1),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'rd_y',rd(2),sim.simx_opmode_streaming);
        sim.simxSetFloatSignal(clientID,'rd_z',rd(3),sim.simx_opmode_streaming);

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

