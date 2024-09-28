function trajectory_function_2(clientID,sim, button_trajectory_2)

% This function executes the wrist trajectory. THETA is controlled to go to
% 2.97 radiants, after that PHI follows a sinusoidal path from -pi/2 to
% pi/2. Press X on the controller to execute the trajectory.
% Press B on the controller to exit the trajectory.

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

    joy = vrjoystick(1);
    exit_flag = false;

    %===INITIALIZATION

    for i=1:7
        [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    qp = qn;
    dq=zeros(7,1);
    J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
    Jp=J;
    rp = EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
    
    for i=1:7
        sim.simxSetJointMaxForce(clientID,h(i),100,sim.simx_opmode_streaming);
    end
    for i=1:7
        sim.simxSetJointForce(clientID,h(i),0,sim.simx_opmode_oneshot);
    end

    %===GAINS

    Km=[250,0,0,0,0,0;
       0,250,0,0,0,0;
       0,0,75,0,0,0;
       0,0,0,45,0,0;
       0,0,0,0,250,0;
       0,0,0,0,0,0];
    Dm=[500,0,0,0,0,0;
       0,500,0,0,0,0;
       0,0,500,0,0,0;
       0,0,0,20,0,0;
       0,0,0,0,650,0;
       0,0,0,0,0,0];
   
    Dq=eye(7)*8;

    %===TRAJECTORIES DEFINITION

    trajectories=4;
    for traj=1:trajectories
        if exit_flag == true
            break
        end
        if traj == 1
            dt=0.05;
            T=20;  
            t=transpose(0:dt:T);
            B=2.98;
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2), ones(length(t),1)*rd(3),(T-t)/T*(rd(4)-B)+ones(length(t),1)*B,ones(length(t),1)*rd(5),zeros(length(t),1)*rd(6)];
            drd = [zeros(length(t),1),zeros(length(t),1), zeros(length(t),1), -(rd(4)-B)/T*ones(length(t),1), zeros(length(t),1), zeros(length(t),1)];
            ddrd = [zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
            
        elseif traj == 2
            for i=1:7
                 [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
            end
            rat = EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
            dt=0.05;
            T=10;  
            t=transpose(0:dt:T);
            B=2.97;
            C=pi/2;
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2), ones(length(t),1)*rd(3),ones(length(t),1)*B,ones(length(t),1)*rd(5),(T-t)/T*(rat(6)-0)];
            drd = [zeros(length(t),1),zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), -rat(6)/T*ones(length(t),1)];
            ddrd = [zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
            
        elseif traj == 3
            dt=0.05;
            T=40;  
            t=transpose(0:dt:T);
            B=2.97;
            C=pi/2;
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2), ones(length(t),1)*rd(3),ones(length(t),1)*B,ones(length(t),1)*rd(5),C*sin(2*pi*t/T)];
            drd = [zeros(length(t),1),zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), C*2*pi/T*cos(2*pi*t/T)];
            ddrd = [zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),-C*(2*pi/T)^2*sin(2*pi*t/T)];

        else
            dt=0.05;
            T=20;  
            t=transpose(0:dt:T);
            B=2.97;
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2), ones(length(t),1)*rd(3),(T-t)/T*(B-rd(4))+ones(length(t),1)*rd(4),ones(length(t),1)*rd(5),zeros(length(t),1)*rd(6)];
            drd = [zeros(length(t),1),zeros(length(t),1), zeros(length(t),1), -(B-rd(4))/T*ones(length(t),1), zeros(length(t),1), zeros(length(t),1)];
            ddrd = [zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
        end

    %=== SIMULATION LOOP

        t = 0;
        time = 1;
        while t<=T
            for i=1:7
                [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
            end
    
            dq=(qn-qp)/dt;
            ra=EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
            dr = (ra-rp)/dt;
            g=get_GravityVector(qn);
            c=get_CoriolisVector(qn,dq);
            M=get_MassMatrix(qn);
            J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
            dJ=(J-Jp)/dt;
    
           [axes,buttons] = read(joy);
            if buttons(2) == 1     %<----PRESS B TO STOP THE EXECUTION                                    
                disp('Trajectory interrupted')
                exit_flag = true;
                break
            end
    
            
            % PHI CONTROL GAINS
            threshold = 0.17;   %PHI control starts at THETA=2.97
            if rd1(time,4)>pi-threshold && rd1(time,4)<pi+threshold
                Km(6,6)=0;
                Dm(6,6)=0;
            else
                Km(6,6)=1;
                Dm(6,6)=1;
            end
            
            % CONTROL LAW
            u=M*pinv(J)*(transpose(ddrd(time,:))-dJ*transpose(dq))+c+g+transpose(J)*(Km*(transpose(rd1(time,:))-ra)+Dm*(transpose(drd(time,:))-dr))-Dq*transpose(dq);
    
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
    
            qp=qn;
            rp=ra;
            Jp=J;
            t = t+dt;
            time = time+1;
            sim.simxSynchronousTrigger(clientID);
        end
        button_trajectory_2.Enable = false;
    end
    disp('Wrist trajectory completed.')
end        

