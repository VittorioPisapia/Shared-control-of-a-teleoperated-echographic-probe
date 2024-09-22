function trajectory_function(clientID,sim)
    %trajectory tracking control law

    %this function returns the current desired task vector in order to make
    %the robot regulation control to not overcome discontinuities
    %OR make a periodic trajectory with end point = start point 
    global rd
    
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
    T=3;
    t=transpose(0:dt:T);

    A=0.1;
    %Trajectory parametrization
    % rd1=[ones(length(t),1)*rd(1)+A*sin((2*pi)*t/T), ones(length(t),1)*rd(2)+A*cos((2*pi)*t/T).*sin((2*pi)*t/T), ones(length(t),1)*rd(3), ones(length(t),1)*rd(4) ,ones(length(t),1)*rd(5), deg2rad(155)*sin((2*pi)/T*t)];
    % drd=[(2*pi)/(T)*A*cos((2*pi)*t/T), (2*pi)/T*A*(cos((2*pi)*t/T).^2-sin((2*pi)*t/T).^2),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),(2*pi)/T*deg2rad(155)*cos((2*pi)/T*t)];
    % ddrd=[-(2*pi)^2/(T^2)*A*sin((2*pi)*t/T), -(2*pi)^2/(T^2)*4*A*cos(t).*sin((2*pi)*t/T), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1),-(2*pi)^2/(T^2)*deg2rad(155)*sin((2*pi)/T*t)];

    rd1 = [ones(length(t),1)*rd(1)+t/T*A,ones(length(t),1)*rd(2)+t/T*A, ones(length(t),1)*rd(3), ones(length(t),1)*rd(4),ones(length(t),1)*rd(5),ones(length(t),1)*rd(6)];
    drd = [ones(length(t),1)*A/T, ones(length(t),1)*A/T, zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1)];
    ddrd = [zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
    %===inizializzazione parametri==========
    for i=1:7
        [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    qp = transpose(qn);
    dq=zeros(7,1);
    Jp=zeros(6,7);
    rp=transpose(rd1(1,:));
    %servono inizializzazione di ddr_precedente e dr_precedente per la derivazione numerica fuori loop
    %==========================================


    %===Spazio per i gain ========

    %=============================
    
    for time=1:length(t)
        disp('t')
        time
        disp('rd1')
        rd1(time,:)
        disp('drd')
        drd(time,:)
        disp('ddrd')
        ddrd(time,:)

        for i=1:7
        	    [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
        end

        dq=(transpose(qn)-qp)/dt;
        disp('ra')
        ra=EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7)) %task attuale
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
            Kphi=0;
            Dphi=0;
        else
            Kphi=1;
            Dphi=1;
        end

        Km=[20,0,0,0,0,0;
           0,20,0,0,0,0;
           0,0,20,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0];
        Dm=[10,0,0,0,0,0;
           0,10,0,0,0,0;
           0,0,10,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0];
        Mm=[50,0,0,0,0,0;
           0,50,0,0,0,0;
           0,0,50,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0];
        Dq=eye(7)*8;
    
        %== possibile forma con check sulla forza di contatto ===========================
        [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_buffer);
        %fz = -force(3);
        
        %control law
        %u=M*pinv(J)*(ddrd(time,:)-dJ*dq+pinv(Mm)*(Dm*(drd(time,:)-dr)+Km*(rd1(time,:)-ra)))+c+g+transpose(J)*(Mr*pinv(Mm)-eye(6)*transpose([force,torque]))-Dq*dq;
         u=M*pinv(J)*(ddrd(time,:)-dJ*dq)+c+g+transpose(J)*(Km*(rd1(time,:)-ra)+Dm*(drd(time,:)-dr))-Dq*dq;

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


