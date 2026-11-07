function cam = qcarCamera()
%#codegen
% QCARCAMERA  Camera settings used by findBlueSign and findRamp.
% These are placeholders. Replace them with the RealSense values from
% Quanser's QCar documentation (or measure them) before trusting distances.

cam.fx = 385;            % focal length in pixels (horizontal)
cam.fy = 385;            % focal length in pixels (vertical)
cam.cx = 320;            % image centre column (640 x 480 image)
cam.cy = 240;            % image centre row
cam.height = 0.17;       % camera height above the road (m)
cam.pitch = 0.0;         % camera tilt downward (rad); 0 = looking straight ahead
cam.forwardOffset = 0.10;% camera position ahead of the car's centre (m)
cam.depthScale = 1.0;    % multiply raw depth values by this to get metres
end