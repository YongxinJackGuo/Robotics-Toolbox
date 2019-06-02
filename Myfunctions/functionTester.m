close all
clear
clc

disp("Test begins..." + newline);
%----quaternion section----
disp("-----------------------Quaternion Operation Section------------------------");
% multiplication
disp("Multiplication: ");
a = [1;2;3;4];
b = [1;3;4;1];
c = Quat_multp(a,b);
disp(c);



%----dual vector section---
disp("----------------------Dual Vector Operation Section------------------------");
% addition
disp("Addition:");
E = [1 2;3 4;7 3];
F = [0 8;3 7;1 2];

G = Dual_vector_add(E,F);
disp(G);
% multiplication between a dual number and a dual vector.
disp("multiplication between a dual number and a dual vector:");
D0 = [2 4];
D0E = Dual_vecAndnum_pro(D0,E);
disp(D0E);
% Dot product between two dual vectors.
disp("Dot product between two dual vectors: ");
EdotF = Dual_vector_dotPro(E,F);
disp(EdotF);
% Cross product between two dual vectors.
disp("Cross product between two dual vectors: ");
EcrossF = Dual_vector_crossPro(E,F);
disp(EcrossF);






%-----dual number section---
disp("---------------------Dual Number Operation Section---------------------")
% addition
disp("addition");
D1 = [3 8];
D2 = [4 7];

D3_add = Dual_number_add(D1,D2);
disp(D3_add);
% multiplication
disp("multiplication");
D3_multp = Dual_number_multp(D1,D2);
disp(D3_multp);
% inverse
disp("Inverse of D1:");
D1_inv = Dual_number_inv(D1);
disp(D1_inv)
% conjugate
disp("Conjugate of D1:");
D1_conj = Dual_number_conj(D1);
disp(D1_conj);
% multiplication of D1 and D1_conj
disp("multiplication of D1 and D1_conj");
D_multp_D1_conj = Dual_number_multp(D1,D1_conj);
disp(D_multp_D1_conj);




%------Dual quaternion section---------
disp("----------------Dual Quaternion Operation Section----------------");
% addition
disp("addition:");
A = [[1;2;5;6],[1;4;5;2]];
B = [[0;3;2;3],[2;0;3;1]];
C = Dual_quat_add(A,B);
disp(C);
% multiplication
disp("multiplication:");
C1 = Dual_quat_multp(A,B);
disp(C1);
% conjugate
disp("conjugate of A");
A_conj = Dual_quat_conj(A);
disp(A_conj);
% unit dual quaternion
disp("Unit dual quaternion test:");
A_unit = [[1;0;0;0],[0;2212;3313;4414]];
A_unit_conj = Dual_quat_conj(A_unit);
unitProd = Dual_quat_multp(A_unit,A_unit_conj);
disp(unitProd);


%------Conversion section--------
disp("---------Dual Quaternion and Transformation Matrix Conversion Section-----------");
% convert from transformation matrix to unit dual quaternion.
disp("convert from transformation matrix to unit dual quaternion:");
roll = -pi/3;
pitch = -pi/7;
yaw = pi/4;
R = RPY_to_Rot([roll,pitch,yaw]);
P = [2;4;12];
g = [R P;[0 0 0],1];
unitDualQuat = Trans_to_UnitDualQuat(g);
disp(unitDualQuat);
% check if it is a unit dual quaternion
disp("check if it is a unit dual quaternion:");
unitDualQuat_conj = Dual_quat_conj(unitDualQuat);
unitDualQuatProd = Dual_quat_multp(unitDualQuat,unitDualQuat_conj);
disp(unitDualQuatProd);
% Convert unit dual quaternion back to the transformation matrix.
disp("convert unit dual quaternion back to the transformation matrix:");
g_back = UnitDualQuat_to_Trans(unitDualQuat);
disp(g_back);
disp("Calculate the difference: ");
diff = g_back - g;
disp(diff);
% Convert pure translation to unit dual quaternion.
disp("Convert pure translation to unit dual quaternion: ");
g_translation = [eye(3),[1;2;3];[0 0 0],1];
A_translation = Trans_to_UnitDualQuat(g_translation);
disp(A_translation);
% Compute the power of a dual quaternion.
disp("Compute the power of a dual quaternion A: ");
disp(newline + "dual quaternion A is: ");
disp(unitDualQuat);
tau = 2;
A_tau = Dual_quat_pow(unitDualQuat,tau);
disp("dual quaternion A raised to the power of " + tau + " is: ");
disp(A_tau);


%-------------SE(3) Interpolation Test Section--------------
disp("--------SE(3) Interpolation Test Section---------");
initial_A = unitDualQuat;
disp("Initial config. is: ");
disp(initial_A);
roll_1 = -pi/5;
pitch_1 = -pi/10;
yaw_1 = pi/6;
R_1 = RPY_to_Rot([roll_1,pitch_1,yaw_1]);
P_1 = [5;8;7];
g_1 = [R_1 P;[0 0 0],1];
final_B = Trans_to_UnitDualQuat(g_1);
disp("Final config. is: ");
disp(final_B);
tau = 1; % specify step size tau.
middle_C = SE3_Interpolation(initial_A,final_B,tau);
disp("At tau = " + tau +", Middle config. is: ")
disp(middle_C);
disp("The final config is: ");
disp(g_1);
% convert middle_C back to transformation matrix representation.
middle_C_trans = UnitDualQuat_to_Trans(middle_C);
disp("The middle_C in transformation matrix representation is: ");
disp(middle_C_trans);
% convert it into RPY angle representation.
R_C_trans = middle_C_trans(1:3,1:3);
rpy = Rot_to_RPY(R_C_trans);
% extract position vector.
P_C_trans = middle_C_trans(1:3,4);
disp("The corresponding RPY angles are: ");
disp(rpy*180/pi);
disp("The corresponding position vector is: ");
disp(P_C_trans);


%--------Main Application File test------------
config_final = [eye(3),[0.6;0.8;0.3];[0 0 0],1];
gst0 = [eye(3),[0;1.5;0.8];[0 0 0],1];
A0_1 = Trans_to_UnitDualQuat(gst0);
Ad_1 = Trans_to_UnitDualQuat(config_final);
tau_1 = 0.01;

disp(Dual_quat_multp(Dual_quat_conj(A0_1),Ad_1));
disp(Dual_quat_pow(Dual_quat_multp(Dual_quat_conj(A0_1),Ad_1),tau_1));
AA = SE3_Interpolation(A0_1,Ad_1,tau_1);
disp(AA);







disp("Test ends...");




