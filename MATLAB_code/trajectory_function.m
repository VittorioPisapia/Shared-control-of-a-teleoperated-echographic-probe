function trajectory_function(clientID,sim, button_trajectory)

% This function executes a linear path trajectory between the two
% keypoints' y position in CoppeliaSim. It is possible to move the dummies in order to
% adjust the path length. Press A on the controller to execute the trajectory.
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
    [r,keypoint1]=sim.simxGetObjectHandle(clientID,'TrajectoryKeypoint1',sim.simx_opmode_blocking);
    [r,keypoint2]=sim.simxGetObjectHandle(clientID,'TrajectoryKeypoint2',sim.simx_opmode_blocking);

    [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_streaming);

    joy = vrjoystick(1);
    exit_flag = false;
    outside_flag = false;

    %===INITIALIZATION 

    for i=1:7
        [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    qp = qn;
    dq=zeros(7,1);
    J=EulerJacobianPose(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
    Jp=J;
    rp = EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7));
    
    %===DUMMY'S POSITION

    [r,pos1]=sim.simxGetObjectPosition(clientID, keypoint1,-1,sim.simx_opmode_blocking);
    [r,pos2]=sim.simxGetObjectPosition(clientID, keypoint2,-1,sim.simx_opmode_blocking);
    y1=pos1(2);
    y2=pos2(2);
    
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

    rp_fixed=rp;
    trajectories=2;
    for traj =1:trajectories
        if exit_flag == true || outside_flag == true
            break
        end
        if traj ==1
            dt=0.05;
            T=40;  
            t=transpose(0:dt:T);
            A=-(rp_fixed(2)-y1);
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2)+A*sin(pi*t/T), ones(length(t),1)*rd(3), ones(length(t),1)*rd(4),ones(length(t),1)*rd(5),ones(length(t),1)*rd(6)];
            drd = [zeros(length(t),1), A*pi/T*cos(pi*t/T), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1)];
            ddrd = [zeros(length(t),1),-A*(pi/T)^2*sin(pi*t/T),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
        else
            dt=0.05;
            T=40;  
            t=transpose(0:dt:T);
            A=-( rp_fixed(2)-y2 );
            rd1 = [ones(length(t),1)*rd(1),ones(length(t),1)*rd(2)+A*sin(pi*t/T), ones(length(t),1)*rd(3), ones(length(t),1)*rd(4),ones(length(t),1)*rd(5),ones(length(t),1)*rd(6)];
            drd = [zeros(length(t),1), A*pi/T*cos(pi*t/T), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zeros(length(t),1)];
            ddrd = [zeros(length(t),1),-A*(pi/T)^2*sin(pi*t/T),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1),zeros(length(t),1)];
        
        end

        %=== SIMULATION LOOP
        
        t = 0;
        time = 1;
        while t<=T

            if rp_fixed(2)<min(y1,y2) || rp_fixed(2)>max(y1,y2)            %initial check
                warning('Y coordinate of the probe is outside the keypoints!')
                outside_flag = true;
                break
            end
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
        button_trajectory.Enable = false;
    end
    if outside_flag == true
        return
    end
    if exit_flag == false
        disp('Linear trajectory completed.')
    end
end


