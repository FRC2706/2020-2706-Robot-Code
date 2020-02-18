package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveBase extends SubsystemBase {
    public abstract PigeonIMU getPigeon();
    public abstract double getCurrentAngle();
    public abstract void setDisabledMode();
    public abstract void arcadeDrive(double forwardVal, double rotateVal, boolean squareInputs);
    public abstract void tankDrive(double leftVal, double rightVal, boolean squareInputs);
    public abstract void curvatureDrive(double forwardSpeed, double curveSpeed, boolean override);
    public abstract void stopMotors();
    public abstract void setOpenLoopVoltage();
    public abstract void setBrakeMode(boolean brake);
    public abstract boolean isBrakeMode();
    public abstract void setSensitiveSteering(boolean enabled);
}
