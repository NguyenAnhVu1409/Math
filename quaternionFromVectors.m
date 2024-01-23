function quaternion = quaternionFromVectors(x_vector, z_vector)
    
   % Chuẩn hóa vector x_vector và vector z_vector
    x_vector = x_vector / norm(x_vector);
    z_vector = z_vector / norm(z_vector);
    
    % Tính toán vector y_vector
    y_vector = cross(z_vector, x_vector);
    y_vector = y_vector / norm(y_vector);
    
    % x_vectorây_vector dựng ma trận x_vectoroay_vector
    rotationMatrix_vector = [x_vector; y_vector; z_vector];
    
    % Chuy_vectorển đổi ma trận x_vectoroay_vector thành quaternion
    quaternion = rotm2quat(rotationMatrix_vector);

end
