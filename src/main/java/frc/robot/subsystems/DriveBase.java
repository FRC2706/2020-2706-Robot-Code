package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

import java.util.function.Consumer;

public abstract class DriveBase extends SubsystemBase {
    // Variables shared among all drive bases
    protected DifferentialDrive differentialDrive;
    protected boolean sensitiveSteering;
    protected PigeonIMU pigeon;
    protected DriveMode driveMode;
    protected NeutralMode neutralMode;
    protected DriveBaseState state;
    
    // This is used to get the pigeon heading.
    private PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    
    // The possible drive modes
    public enum DriveMode {
        // There is no control mode active
        Disabled,
        
        // Standard open loop voltage control
        OpenLoopVoltage
    }
    
    public enum DriveBaseState {
        Fine,
        Degraded,
        Unusable
    }
    
    /**
     * Motor control method for arcade drive.
     * @param forwardVal The forward value
     * @param rotateVal The rotate value
     * @param squareInputs Square the inputs?
     */
    public final void arcadeDrive(double forwardVal, double rotateVal, boolean squareInputs) {
        this.setDriveMode(DriveMode.OpenLoopVoltage);
        differentialDrive.arcadeDrive(forwardVal, rotateVal*Config.DRIVETRAIN_DEFAULT_MAX_SPEED.get(), squareInputs);
        this.followMotors();
    }
    
    /**
     * Motor control method for tank drive.
     * @param leftVal The left value
     * @param rightVal The right value
     * @param squareInputs Square the inputs?
     */
    public final void tankDrive(double leftVal, double rightVal, boolean squareInputs) {
        this.setDriveMode(DriveMode.OpenLoopVoltage);
        differentialDrive.tankDrive(leftVal, rightVal, squareInputs);
        this.followMotors();
    }
    
    /**
     * Motor control method for curvature drive
     * @param forwardSpeed The forward speed
     * @param curveSpeed The curve speed
     * @param override Override?
     */
    public final void curvatureDrive(double forwardSpeed, double curveSpeed, boolean override) {
        this.setDriveMode(DriveMode.OpenLoopVoltage);
        differentialDrive.curvatureDrive(forwardSpeed, curveSpeed, override);
        this.followMotors();
    }
    
    /**
     * Sets if sensitive steering is enabled
     * @param enabled?
     */
    public final void setSensitiveSteering(boolean enabled) {
        sensitiveSteering = enabled;
    }
    
    /**
     * Returns true if the pigeon has been defined
     * @return True if the pigeon is defined, false otherwise
     */
    public final boolean hasPigeon() {
        return pigeon != null;
    }
    
    /**
     * A getter for the pigeon
     * @return The pigeon
     */
    public final PigeonIMU getPigeon() {
        return pigeon;
    }
    
    /**
     * Tries to get the current angle as reported by the pigeon
     * @return The current heading (In degrees) or 0 if there is no pigeon.
     */
    public final double getCurrentAngle() {
        if (!hasPigeon()) return 0d;
        pigeon.getFusedHeading(fusionStatus);
        return fusionStatus.heading;
    }
    
    /**
     * This should be implemented by the drive base. It will stop all the motors.
     */
    public abstract void stopMotors();
    /**
     * This should be impmenented by the drive base. It will reset all the motors.
     */
    protected abstract void resetMotors();
    
    /**
     * This sets the drive mode for the drive base
     * @param mode The mode to use
     */
    public final void setDriveMode(DriveMode mode) {
        if (driveMode != mode) driveModeUpdated(mode);
        driveMode = mode;
    }
    
    /**
     * This should be implemented by the drive base. It will handle setting the neutral mode.
     * @param mode The mode to use
     */
    public final void setNeutralMode(NeutralMode mode) {
        if (neutralMode != mode) neutralModeUpdated(mode);
        neutralMode = mode;
    }
    
    /**
     *
     */
    public final NeutralMode getNeutralMode() {
        return neutralMode;
    }
    
    /**
     * This lets the drive base set their motors to follow
     */
    protected abstract void followMotors();
    
    /**
     * This is a callback for the drive base to update their motors with the new drive mode.
     * It's not required to override this if you don't need it.
     * @param driveMode the mode we switched into
     */
    protected void driveModeUpdated(DriveMode driveMode) {

    }

    /**
     * Get the distance travelled by the left encoder in encoder ticks
     *
     * @return The distance of the right encoder (0 before overide)
     */
    public double getLeftDistance(){
        return 0;
    }

    /**
     * Get the distance travelled by the right encoder in encoder ticks
     *
     * @return The distance of the right encoder (0 before overide)
     */
    public double getRightDistance() {
        return 0;
    }

    /**
     * Get the speed of the left encoder in encoder ticks per second
     *
     * @return The speed of the left encoder (0 before overide)
     */
    public double getLeftSpeed() {
        return 0;
    }

    /**
     * Get the speed of the right encoder in encoder ticks per second
     *
     * @return The speed of the right encoder (0 before overide)
     */
    public double getRightSpeed() {
        return 0;
    }

    //Reset encoder values
    public void resetEncoders() {

    }
    
    /**
     * This is a callback for the drive base to update their motors with the new neutral mode.
     * It's not required to override this if you don't need it.
     * @param neutralMode the mode we switched into
     */
    protected void neutralModeUpdated(NeutralMode neutralMode) {
    
    }

    /**
     * Measured distance units for converting ticks to units or units to encoder ticks
     * 
     */
    public enum DistanceUnit {

        //Encoder tick is equal to itself
        ENCODER_TICKS(1d),
        
        //Measured value of 8583 ticks per meter
        METERS(8583.1d),

        //1 cm = 0.01 meters. 8583 * 0.01 = 858.3. Rounding to 858 ticks.
        CENTIMETERS(858.1d),

        //1 foot is equal to 0.3048 meters. 8583 * 0.3048 = 2616.1 (approx). Rounding to 2616 ticks per foot
        FEET(2616.1d),

        //39.3701 inches is equal to 1 meter. 8583 / 39.3701 = 218.008 (approx). Rounding to 218 ticks per inch
        INCHES(218.1d);

        //Update encoder ticks with unit
        public double encoderTicks;
        DistanceUnit(double encoderTicks) {
            this.encoderTicks = encoderTicks;
        }
    }

    /**
     * Distance unts to travel 
     * 
     */
    public enum DistanceType {

        //Different distance units
        CENTIMETERS, METERS, INCHES, FEET; 

    }
}
