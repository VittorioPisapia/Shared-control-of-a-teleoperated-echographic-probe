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
    rd=[rd(1)+A*sin(t);rd(2)+A*cos(t)*sin(t);rd(3);rd(4);rd(5);rd(6)];
    drd=[A*cos(t);A*(cos(t)^2-sin(t)^2);0;0;0;0];
    ddrd=[-A*sin(t);-4*A*cos(t)*sin(t);0;0;0;0];

    %===inizializzazione parametri==========
    for i=1:7
        [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
    end
    qp = qn;
    dq=zeros(7);
    %servono inizializzazione di ddr_precedente e dr_precedente per la derivazione numerica fuori loop
    %==========================================


    %===Spazio per i gain ========

    %=============================
    
    for time=1:length(t)
        for i=1:7
        	    [r,qn(i)]=sim.simxGetJointPosition(clientID,h(i),sim.simx_opmode_streaming);
        end

        dq=(qn-qp)/dt;
        ra=EulerTaskVector(qn(1),qn(2),qn(3),qn(4),qn(5),qn(6),qn(7)); %task attuale
        dr = (ra-rp)/dt;

        g=get_GravityVector(qn);
        c=get_CoriolisVector(qn,dq);
        M=get_MassMatrix(qn);
    
        %== possibile forma con check sulla forza di contatto ===========================
        [r, state, force, torque] = sim.simxReadForceSensor(clientID, ForceSensor, sim.simx_opmode_buffer);
        fz = -force(3);
        while true
            if fz < 5
                rd(3) = rd(3)-0.01;
            else
                break;
            end
        end  
        %============================================    
        sim.simxSynchronousTrigger(clientID);
    end

    % u=0;
    % 
    % r=EulerTaskVector(q1,q2,q3,q4,q5,q6,q7);
end


