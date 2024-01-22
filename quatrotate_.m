function rotatedVector = quatrotate_(quaternion, vector)
    % Chuẩn hóa vector đầu vào
    vector = vector(:)';
    
    % Chuẩn hóa quaternion đầu vào
    quaternion = quaternion(:)';
    quaternion = quaternion / norm(quaternion);

    % Tạo ma trận quay từ quaternion
    q0 = quaternion(1);
    q1 = quaternion(2);
    q2 = quaternion(3);
    q3 = quaternion(4);

    % Cách 1
    % R = [q0^2 + q1^2 - q2^2 - q3^2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2;
    %      2*q1*q2 + 2*q0*q3, q0^2 - q1^2 + q2^2 - q3^2, 2*q2*q3 - 2*q0*q1;
    %      2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, q0^2 - q1^2 - q2^2 + q3^2];
    
    % Cách 2
    % R = [1 - 2*q2^2 - 2*q3^2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2;
    %      2*q1*q2 + 2*q0*q3, 1 - 2*q1^2 - 2*q3^2, 2*q2*q3 - 2*q0*q1;
    %      2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1^2 - 2*q2^2];

    % Xoay vector bằng ma trận xoay
    % rotatedVector = vector * R;

    % Cách 3
     quatvector = quatmultiply(quaternion, quatmultiply([0,vector],quatconj(quaternion)));
     rotatedVector(1) = quatvector(2);
     rotatedVector(2) = quatvector(3);
     rotatedVector(3) = quatvector(4);

end