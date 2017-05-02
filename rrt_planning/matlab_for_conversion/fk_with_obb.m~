clear all
load('Baxter_kinetic_output.mat')

%Defining boxes:
C = zeros(3,4);
V = zeros(4,3,4);
E = zeros(3,4);
%Box 1:
C(:,1) = [0;0;0.27035/2];
V(:,1,1) = [1;0;0;0];
V(:,2,1) = [0;1;0;0];
V(:,3,1) = [0;0;1;0];
E(:,1) = [.060; 0.060; 0.27035/2];

%Translate into local coordinate frame:
box{1} = [1 0 0 0;...
          0 1 0 0;...
          0 0 1 0.27035/2;...
          0 0 0 1];

%Box 2:
C(:,2) = [0;0.069+0.36445/2; 0.27035];
V(:,1,2) = [1;0;0;0];
V(:,2,2) = [0;1;0;0];
V(:,3,2) = [0;0;1;0];
E(:,2) = [0.06; 0.360/2; 0.06];

%Translate into local coordinate frame:
box{2} = [1 0 0 0;...
          0 1 0 0.069+0.36445/2;...
          0 0 1 0.27035;...
          0 0 0 1];

%Box 3:
C(:,3) = [0;0.069+0.36435+0.37429/2;0.27035-0.069];
V(:,1,3) = [1;0;0;0];
V(:,2,3) = [0;1;0;0];
V(:,3,3) = [0;0;1;0];
E(:,3) = [0.06; 0.370/2; 0.06];

%Translate into local coordinate frame:
box{3} = [1 0 0 0;...
          0 1 0 0.069+0.36435+0.37429/2;...
          0 0 1 0.27035-0.069;...
          0 0 0 1];
          

%Box 4:
C(:,4) = [0;0.069+0.36435+0.37429+.229525/2;0.27035-0.069-0.01];
V(:,1,4) = [1;0;0;0];
V(:,2,4) = [0;1;0;0];
V(:,3,4) = [0;0;1;0];
E(:,4) = [0.06; 0.220/2; 0.06];

%Translate into local coordinate frame:
box{4} = [1 0 0 0;...
          0 1 0 0.069+0.36435+0.37429+.229525/2;...
          0 0 1 0.27035-0.069-0.01;...
          0 0 0 1];
      
%Object in space:
% Co = [0;0.069+0.036435+0.37429+0.1;0.27035];
Co = [0;0.069+0.36435+0.37429+.229525/2;0.27035-0.069-0.01];
Vo(:,1) = [1;0;0;0];
Vo(:,2) = [0;1;0;0];
Vo(:,3) = [0;0;1;0];
Eo(:,1) = [0.2;10;10];

object{1} = [1 0 0 0;...
             0 1 0 0.069+0.36435+0.37429+.229525/2;...
             0 0 1 0.27035-0.069-0.01;...
             0 0 0 1];

%check box 4 against object in space:
% collideCheck_4o = collision_checker(C(:,4),Co,V(:,:,4),Vo,E(:,4),Eo);
%use function that takes local coord frames:
disp('Box 4 object collision check: ')
colCheck(box{4}, E(:,4), object{1}, Eo);

%check box 1 against object in space:
% collideCheck_1o = collision_checker(C(:,1),Co,V(:,:,1),Vo,E(:,1),Eo)
%use function that takes local coord frames:
disp('Box 1 object collision check: ')
colCheck(box{1}, E(:,1), object{1}, Eo);

%Rotate box 4:
T1 = -pi/2;
T2 = 0;
T3 = 0;
T4 = 0;
T5 = 0;
T6 = 0;
T7 = 0;

rotated_box4 = eval(custom_cum_prod(exp_twist,1,7))*box{4};
disp('Box 4 rotated object collision check: ')
colCheck(rotated_box4, E(:,4), object{1}, Eo);
