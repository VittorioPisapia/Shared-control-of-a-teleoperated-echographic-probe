function Copy_of_trajectory_function(clientID,sim)
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

    %===inizializzazione parametri==========
    for i=1:7
        [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    qp = qn;
    dq=zeros(7,1);
    J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
    Jp=J;
    disp('disp r precedente  fuori for')
    rp = EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7))
    disp('disp rd passato')
    rd
    
    for i=1:7
        sim.simxSetJointMaxForce(clientID,h(i),100,sim.simx_opmode_streaming);
    end
    for i=1:7
        sim.simxSetJointForce(clientID,h(i),0,sim.simx_opmode_oneshot);
    end

    %servono inizializzazione di ddr_precedente e dr_precedente per la derivazione numerica fuori loop
    %==========================================


    %===Spazio per i gain ========

        Km=[250,0,0,0,0,0;
           0,250,0,0,0,0;
           0,0,75,0,0,0;
           0,0,0,45,0,0;
           0,0,0,0,250,0;
           0,0,0,0,0,0];
        Dm=[500,0,0,0,0,0;
           0,500,0,0,0,0;
           0,0,500,0,0,0;
           0,0,0,10,0,0;
           0,0,0,0,650,0;
           0,0,0,0,0,0];
       
        Dq=eye(7)*0;

    %=============================
    trajectories=2;
    for traj=1:trajectories
        if traj ==1
            dt=0.05;
            T=60;  
            t=transpose(0:dt:T);
            A=0.08;
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2)+A*sin(2*pi*t/T), ones(length(t),1)*rd(3), ones(length(t),1)*rd(4),ones(length(t),1)*rd(5),ones(length(t),1)*rd(6)];
            drd = [zeros(length(t),1), A*2*pi/T*cos(2*pi*t/T), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1)];
            ddrd = [zeros(length(t),1),-A*(2*pi/T)^2*sin(2*pi*t/T),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
        
        else
            dt=0.05;
            T=40;  
            t=transpose(0:dt:T);
            B=2.97;
            C=pi/2;
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2), ones(length(t),1)*rd(3),ones(length(t),1)*B,ones(length(t),1)*rd(5),ones(length(t),1)*rd(6)+C*sin(2*pi*t/T)];
            drd = [zeros(length(t),1),zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), C*2*pi/T*cos(2*pi*t/T)];
            ddrd = [zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),-C*(2*pi/T)^2*sin(2*pi*t/T)];

        %    t_1=10;
        %    t_2=T-t_1;
        %    for t_t =1:length(t)
        % 
        %        if t_t<t_1 
        %            rd1(t_t, 4)=interp1([0, t_1], [rd(4),B], t_t);
        %            drd(t_t,4)=-(pi-B)/t_1;
        % 
        %        elseif t_t>=t_1 && t_t<t_2
        %            rd1(t_t, 4)=B;
        % 
        %        else
        %            rd1(t_t, 4)=interp1([t_2, T], [B,rd(4)], t_t);
        %            drd(t_t,4)=-(B-pi)/t_2;
        %        end
        %    end
        % end
        t = 0;
        time = 1;
        while t<=T
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
    
            dr = (ra-rp)/dt;
    
            g=get_GravityVector(qn);
            c=get_CoriolisVector(qn,dq);
            M=get_MassMatrix(qn);
            J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
            dJ=(J-Jp)/dt;
    
            
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
            
            %control law
            %u=M*pinv(J)*(ddrd(time,:)-dJ*dq+pinv(Mm)*(Dm*(drd(time,:)-dr)+Km*(rd1(time,:)-ra)))+c+g+transpose(J)*(Mr*pinv(Mm)-eye(6)*transpose([force,torque]))-Dq*dq;
            u=M*pinv(J)*(transpose(ddrd(time,:))-dJ*transpose(dq))+c+g+transpose(J)*(Km*(transpose(rd1(time,:))-ra)+Dm*(transpose(drd(time,:))-dr))-Dq*transpose(dq);
            % u=M*pinv(J)*(-dJ*dq)+c+g+transpose(J)*(Km*(rd-ra)+Dm*(-dr))-Dq*dq;
            % u=M*pinv(J)*(ddrd-dJ*transpose(dq))+c+g+transpose(J)*(Km*(rd1-ra)+Dm*(drd-dr))-Dq*transpose(dq); %main
    
    
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
            % pause();
            
            qp=qn;
            rp=ra;
            Jp=J;
            t = t+dt;
            time = time+1;
            sim.simxSynchronousTrigger(clientID);
        end
    end

end

