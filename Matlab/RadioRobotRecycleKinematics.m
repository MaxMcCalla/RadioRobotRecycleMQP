%

%Set up variables
syms theta1 theta2 theta3 theta4

d1 = 1000;
d6 = 200;

c1 = cos(theta1);
s1 = sin(theta1);
c2 = cos(theta2);
s2 = sin(theta2);
c3 = cos(theta3);
s3 = sin(theta3);
c4 = cos(theta4);
s4 = sin(theta4);

DHTable = [0 5.38 0 0
           theta1 5.4 0 deg2rad(90)
           theta2+deg2rad(90) 0 14.5 0
           theta3 0 0 deg2rad(90)
           theta4 13 0 0
           ];

NewDHTable = [0 5.38 0 0
           theta1 5.4 0 deg2rad(90)
           theta2+deg2rad(90) 0 14.5 0
           theta3 0 0 deg2rad(90)
           theta4 13 0 0
           ];


function T = DHParam(params)
theta = params(1);
d = params(2);
a = params(3);
alpha = params(4);
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
     0          sin(alpha)             cos(alpha)            d
     0          0                      0                     1];
end

%Create the inbetween fk matrices for the jacobian
T1 = DHParam(DHTable(1,:));

T2 = DHParam(DHTable(2,:));

T3 = DHParam(DHTable(3,:));

T4 = DHParam(DHTable(4,:));

T5 = DHParam(DHTable(5,:));

T = T1 * T2 * T3 * T4 * T5;

%Make a callable fk function
function P = fk(theta,T4)
syms theta1 theta2 theta3 theta4
P = subs(T4,[theta1, theta2, theta3, theta4],theta);
end


%Define the jacobian
j1 = [diff(T(1,4),theta1),diff(T(2,4),theta1),diff(T(3,4),theta1),T2(3,1),T2(3,2),T2(3,3)]';
j2 = [diff(T(1,4),theta2),diff(T(2,4),theta2),diff(T(3,4),theta2),T3(3,1),T3(3,2),T3(3,3)]';
j3 = [diff(T(1,4),theta3),diff(T(2,4),theta3),diff(T(3,4),theta3),T4(3,1),T4(3,2),T4(3,3)]';
j4 = [diff(T(1,4),theta4),diff(T(2,4),theta4),diff(T(3,4),theta4),T5(3,1),T5(3,2),T5(3,3)]';

J = [j1 j2 j3 j4];

%Setup the initial variables and constants for IK
function q = IK(pd,T,J)
syms theta1 theta2 theta3 theta4

q0 = [0,-0.5,0,0];
i=1;

q = q0;
e = 0.001;

t = fk(q,T);

%Run the numerical IK
while(norm(pd-t(1:3,4))>e)
    %whos
    Jt = subs(J,[theta1, theta2, theta3, theta4],round(q,25));
    t = fk(q,T);

    dq=pinv(Jt(1:3,:)) * (pd-(t(1:3,4)));
    dq = round(dq,25);
    q = q+dq';  
    %disp(q)
    %disp(round(norm(pd-t(1:3,4))))
end


%Joint Angles
%q

end

function writeJointValues(j)
serials = serialportlist();
    for index = 1:length(serials)  
        target = serials(index);
        try
            output = serialport(target,115200);
            break;
        catch exception
            disp(target);
        end
    end
    for i=1:5
        send = string(i) + string(round(rad2deg(j(i)),5)) + ">";
        write(output, send, "string");
        pause(0.2);
    end
    %Gripper State
    send = string(i) + string(j(6)) + ">";
    write(output, send, "string");
    pause(0.2);
end




round(fk([0,0,0,0],T),4)

allMat = [];
node = ros2node("/robot");
pose_publisher = ros2publisher(node, '/robot_pose', 'geometry_msgs/Transform');

while(true)
spaceSet = false;
space = input("Select World(1) or Joint(2) space");
while(spaceSet == false)
    if space == 1
        inX = input("Select X");
        inY = input("Select Y");
        inZ = input("Select Z");
        j4 = deg2rad(input("Select Wrist Tilt"));
        j5 = input("Select Gripper State");
        joints = [IK([inX,inY,inZ]',T,J),j4,j5];
        disp("Joint Values:")
        for index = 1:5
            disp(round(rad2deg(joints(index))))
        end
            disp(joints(6))
        spaceSet = true;
    elseif space == 2
        j0 = deg2rad(input("Select Base Angle"));
        j1 = deg2rad(input("Select Shoulder Angle"));
        j2 = deg2rad(input("Select Elbow Angle"));
        j3 = deg2rad(input("Select Wrist Angle"));
        j4 = deg2rad(input("Select Wrist Tilt"));
        j5 = input("Select Gripper State");
        joints = [j0, j1, j2, j3, j4, j5];
        spaceSet = true;
    else
        disp("invalid input")
        space = input("Select World(1) or Joint(2) space");
    end
end
%disp(round(rad2deg(joints(2))))
%disp(round(rad2deg(joints(3))))
test1 = round(fk(joints(1:4),T1));
test2 = round(fk(joints(1:4),T1*T2));
test3 = round(fk(joints(1:4),T1*T2*T3));
test4 = round(fk(joints(1:4),T1*T2*T3*T4));
test5 = round(fk(joints(1:4),T),4);
%allMat = [allMat;test5];
%allMat
%test4
x = [test1(1:3,4),test2(1:3,4),test3(1:3,4),test4(1:3,4),test5(1:3,4)];
plot3(x(1,:),x(2,:),x(3,:))
writeJointValues(joints)
xlim([0,30])
ylim([-25,25])
zlim([0,40])
msg = ros2message(pose_publisher);
test5
inv(test5)
transMsg = double(inv(test5));
msg.translation.x = transMsg(1, 4);
msg.translation.y = transMsg(2, 4);
msg.translation.z = transMsg(3, 4);
quat = quatnormalize(rotm2quat(double(transMsg(1:3, 1:3))));
msg.rotation.x = quat(1);
msg.rotation.y = quat(2);
msg.rotation.z = quat(3);
msg.rotation.w = quat(4);
send(pose_publisher, msg);

end


Verification with FK
%round(fk(q,T),5)