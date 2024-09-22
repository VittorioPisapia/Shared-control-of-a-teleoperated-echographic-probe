function trajectory_function(clientID,sim,rd)
    %trajectory tracking control law

    %this function returns the current desired task vector in order to make
    %the robot regulation control to not overcome discontinuities
    %OR make a periodic trajectory with end point = start point 
    
    
    h=zeros(1,6);
    [r,h(1)]=sim.simxGetObjectHandle(clientID,'Franka_joint1',sim.simx_opmode_blocking);
    [r,h(2)]=sim.simxGetObjectHandle(clientID,'Franka_joint2',sim.simx_opmode_blocking);
    [r,h(3)]=sim.simxGetObjectHandle(clientID,'Franka_joint3',sim.simx_opmode_blocking);
    [r,h(4)]=sim.simxGetObjectHandle(clientID,'Franka_joint4',sim.simx_opmode_blocking);
    [r,h(5)]=sim.simxGetObjectHandle(clientID,'Franka_joint5',sim.simx_opmode_blocking);
    [r,h(6)]=sim.simxGetObjectHandle(clientID,'Franka_joint6',sim.simx_opmode_blocking);
    [r,h(7)]=sim.simxGetObjectHandle(clientID,'Franka_joint7',sim.simx_opmode_blocking);
    [r,ForceSensor]=sim.simxGetObjectHandle(clientID,'Franka_connection',sim.simx_opmode_blocking);

    [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_streaming);

    %Simulation time
    dt=0.05;
    T=2;
    t=0:dt:T;

    A=1;
    %Trajectory parametrization
    rd=[rd(1)+A*sin(2*pi*t),rd(2)+A*cos(2*pi*t)*sin(2*pi*t),once(length(t),1)*rd(3),once(length(t),1)*rd(4),once(length(t),1)*rd(5),deg2rad(155)*sin(2*pi*t)];
    drd=[2*pi*A*cos(2*pi*t),2*pi*A*(cos(2*pi*t)^2-sin(2*pi*t)^2),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),2*pi*deg2rad(155)*cos(2*pi*t)];
    ddrd=[-2*pi*A*sin(2*pi*t),-2*pi*4*A*cos(t)*sin(2*pi*t),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),-2*pi*deg2rad(155)*sin(2*pi*t)];

    %===inizializzazione parametri==========
    for i=1:7
        [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    qp = transpose(qn);
    dq=zeros(7,1);
    Jp=zeros(6,7);
    rp=transpose(rd(1,:));
    %servono inizializzazione di ddr_precedente e dr_precedente per la derivazione numerica fuori loop
    %==========================================


    %===Spazio per i gain ========

    %=============================
    
    for time=1:length(t)
        for i=1:7
        	    [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
        end

        dq=(transpose(qn)-qp)/dt;
        ra=EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7)); %task attuale
        dr = (ra-rp)/dt;

        g=get_GravityVector(qn);
        c=get_CoriolisVector(qn,dq);
        M=get_MassMatrix(qn);
        J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
        dJ=(J-Jp)/dt;

        Mr=pinv(J*pinv(M)*transpose(J));

        %% GAINS
        threshold = 0.17;   %PHI control starts at THETA=2.97
        if rd(4)>pi-threshold && rd(4)<pi+threshold
            Kphi=0;
            Dphi=0;
        else
            Kphi=1;
            Dphi=1;
        end

        Km=[250,0,0,0,0,0;
           0,250,0,0,0,0;
           0,0,75,0,0,0;
           0,0,0,45,0,0;
           0,0,0,0,250,0;
           0,0,0,0,0,Kphi];
        Dm=[500,0,0,0,0,0;
           0,500,0,0,0,0;
           0,0,500,0,0,0;
           0,0,0,8,0,0;
           0,0,0,0,650,0;
           0,0,0,0,0,Dphi];
        Mm=[50,0,0,0,0,0;
           0,50,0,0,0,0;
           0,0,500,0,0,0;
           0,0,0,250,0,0;
           0,0,0,0,50,0;
           0,0,0,0,0,500];
        Dq=eye(7)*8;
    
        %== possibile forma con check sulla forza di contatto ===========================
        [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_buffer);
        %fz = -force(3);
        
        %control law
        u=M*pinv(J)*(ddrd(time)-dJ*dq+pinv(Mm)*(Dm*(drd(time)-dr)+Km*(rd(time)-ra)))+c+g+transpose(J)*(Mr*pinv(Mm)-eye(6)*transpose([force,torque]))-Dq*dq;
        

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


        % while true
        %     if fz < 5
        %         rd(3) = rd(3)-0.01;
        %     else
        %         break;
        %     end
        % end  
        %============================================    
        sim.simxSynchronousTrigger(clientID);

        rp=ra;
        Jp=J;
    end

end


