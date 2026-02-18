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
           theta1 5.4 2 deg2rad(90)
           theta2+deg2rad(90) 0 14.5 0
           theta3 0 2 deg2rad(90)
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

q0 = [0,0,0,0];
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
end


%Joint Angles
q

end

function writeJointValues(j)
    for index = 0:10  
        target = "/dev/ttyUSB" + index;
        try
            output = serialport(target,115200);
            break;
        catch exception
            disp(target)
        end
    end
    for i=1:4
        send = string(i) + string(round(rad2deg(j(i)),5)) + ">";
        write(output, send, "string");
        pause(0.2);
    end
end






while(true)
inX = input("Select X")
inY = input("Select Y")
inZ = input("Select Z")
joints = IK([inX,inY,inZ]',T,J)
writeJointValues(joints)

end


%Verification with FK
%round(fk(q,T),5)