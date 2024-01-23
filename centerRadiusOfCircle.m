function [centerPosition,radius,qVector] = centerRadiusOfCircle(startPosition,endPosition,midPosition)
    
    % Tìm trung diem M
 pointM = (startPosition + midPosition) / 2;
    % Tìm trung diem N
 pointN = (endPosition + midPosition) / 2;

 % Tim phương trình mặt phẳng apha đi qua M
    % Vector pháp tuyến n (startPosition-midPosition)
    nVector = (startPosition - midPosition) ;
    nVector_4 = dot(pointM, nVector);
    % fprintf('Phương trình mặt phẳng: %.2fx + %.2fy + %.2fz + %.2f = 0\n', nVector(1), nVector(2), nVector(3), nVector_4);

 % Tim phương trình mặt phẳng beta đi qua N
    % Vector pháp tuyến v (endPosition-midPosition)
    vVector = (endPosition - midPosition) ;
    vVector_4 = dot(pointN, vVector);
    % fprintf('Phương trình mặt phẳng: %.2fx + %.2fy + %.2fz + %.2f = 0\n', vVector(1), vVector(2), vVector(3), vVector_4);

  % Tim phương trình mặt phẳng gama đi qua 3 diem
    % Vector pháp tuyến q (startPosition-endPosition-midPosition)
    qVector = cross(nVector, vVector);
    qVector_4 = dot(midPosition, qVector);
    % fprintf('Phương trình mặt phẳng: %.2fx + %.2fy + %.2fz + %.2f = 0\n', qVector(1), qVector(2), qVector(3), qVector_4);

 A = [nVector(1) nVector(2) nVector(3);
      vVector(1) vVector(2) vVector(3);
      qVector(1) qVector(2) qVector(3)];

 B = [nVector_4; vVector_4; qVector_4];

 % Tọa độ điểm tâm cung tròn
 centerPosition = (A\B)';
 % Bán kính cung tròn
 radius = norm(centerPosition - startPosition);
 qVector = qVector/norm(qVector);
end