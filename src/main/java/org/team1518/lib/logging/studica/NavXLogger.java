package org.team1518.lib.logging.studica;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(AHRS.class)
public class NavXLogger extends ClassSpecificLogger<AHRS> {

    public NavXLogger() {
        super(AHRS.class);
    }

    @Override
    public void update(EpilogueBackend backend, AHRS imu) {
        backend.log("accelerationX", imu.getWorldLinearAccelX());
        backend.log("accelerationY", imu.getWorldLinearAccelY());
        backend.log("accelerationZ", imu.getWorldLinearAccelZ());
        backend.log("velocityX", imu.getVelocityX());
        backend.log("velocityY", imu.getVelocityY());
        backend.log("velocityZ", imu.getVelocityZ());
        backend.log("yaw", imu.getYaw());
        backend.log("pitch", imu.getPitch());
        backend.log("roll", imu.getRoll());
        backend.log("connected", imu.isConnected());
    }
}
