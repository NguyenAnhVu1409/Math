clc;clear;

% Điểm bắt đầu A
A = [-4,0,0];
% Điểm kết thúc B
B = [0,4,0];
% Điểm trung gian C
C = [4,0,0];
% Tìm tâm và bán kính cung tròn
[O,R,qVector] = centerRadiusOfCircle(A,B,C);
% Đặt hệ trục tọa độ tại tâm xoay O
% Thành phần tịnh tiến bằng O = []
% Thành phần quay
vector_x = (A - O)/norm(A-O);
vector_z = qVector/norm(qVector);
vector_end = (B - O)/norm(B-O);
theta = acos(dot(vector_x, vector_end));                % truong hop goc be
theta = 2*pi - acos(dot(vector_x, vector_end));      % truong hop goc lon

q_rotate = quaternionFromVectors(vector_x,vector_z)
q_Init = [1,0,0,0];
q_End = quatnormalize([cos(theta/2),0,0,sin(theta/2)])
Q = quatmultiply(q_rotate,q_End)
Position_Interpolted_User= quatrotate_(Q,A) + O


% figure;
% hold on;
% quiver3(O(1), O(2), O(3), vector_x(1), vector_x(2), vector_x(3), 'r');
% % quiver3(O(1), O(2), O(3), Global_y(1), Global_y(2), Global_y(3), 'g');
% quiver3(O(1), O(2), O(3), vector_z(1), vector_z(2), vector_z(3), 'b');
% view(3);                % Hiển thị dưới dạng xem 3D
% axis equal;             % Fix khung hình
% grid on;
