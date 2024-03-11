package frc.robot.subsystems.Gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import lib.team3526.constants.CTRECANDevice;

public class GyroIOPigeon implements GyroIO {
    public Pigeon2 gyro;

    public GyroIOPigeon(CTRECANDevice device) {
        gyro = new Pigeon2(device.getDeviceID(), device.getCanbus());
    }

    public double getPitch() {
        return gyro.getPitch().refresh().getValue();
    }

    public double getYaw() {
        return -gyro.getAngle();
    }

    public double getRoll() {
        return gyro.getRoll().refresh().getValue();
    }

    public double getPitchVelocity() {
        return gyro.getAngularVelocityXWorld().refresh().getValue();
    }

    public double getYawVelocity() {
        return gyro.getRate();
    }

    public double getRollVelocity() {
        return gyro.getAngularVelocityZWorld().refresh().getValue();
    }

    public double getAccelerationX() {
        return gyro.getAccelerationX().refresh().getValue();
    }

    public double getAccelerationY() {
        return gyro.getAccelerationY().refresh().getValue();
    }

    public double getAccelerationZ() {
        return gyro.getAccelerationZ().refresh().getValue();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }
    
    public void reset() {
        gyro.reset();
    }

    public double getVelocityX() {
        // TODO Auto-generated method stub
        return 0;
    }

    public double getVelocityY() {
        // TODO Auto-generated method stub
        return 0;
    }

    public double getVelocityZ() {
        // TODO Auto-generated method stub
        return 0;
    }
}
