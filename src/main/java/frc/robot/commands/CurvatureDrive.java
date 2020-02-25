package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

/**
 * Abstract class to extend when using curve drive, allows for basic Command architecture
 * Allows the WPI Curvature Drive to be used with passed values
 */
public abstract class CurvatureDrive extends CommandBase {
    private final Supplier<Double> forwardVal;
    private final Supplier<Double> curveSpeed;
    private final boolean initBrake;
    private final Supplier<Boolean> buttonPress;
    private final boolean squareInputs;
    private final DriveBase driveBase;


    private static final double forwardSpeedMultiplier = 0.6;
    private static final double breakButtonRotationSpeedDivisor = 2.5;
    private static final double passiveRotationSpeedDivisor = 2;

    /**
     * Creates the curvature drive
     *
     * @param forwardVal The values to use for driving forward
     * @param curveSpeed The amount that the robot should curve while driving
     * @param initBrake  Whether to start and end the command in brake or coast mode
     */
    protected CurvatureDrive(Supplier<Double> forwardVal, Supplier<Double> curveSpeed,
                             boolean initBrake, Supplier<Boolean> buttonPress, boolean squareInputs) {
        /*
           Ensure that this command is the only one to run on the drive base
           Requires must be included to use this command as a default command for the drive base
        */
        this.driveBase = DriveBaseHolder.getInstance();
        this.forwardVal = forwardVal;
        this.curveSpeed = curveSpeed;
        this.initBrake = initBrake;
        this.buttonPress = buttonPress;
        this.squareInputs = squareInputs;
        addRequirements(this.driveBase);
    }

    @Override
    public void initialize() {
        // Prepare for driving by human
        this.driveBase.setDriveMode(DriveBase.DriveMode.OpenLoopVoltage);
        this.driveBase.setNeutralMode(initBrake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void execute() {
        // Makes the robot move
        double forward = forwardVal.get();
        double curve = curveSpeed.get();

        if (squareInputs) {
            curve *= curve < 0 ? -curve : curve;
        }

        double rotation = Math.abs(curve) < Config.CONTROLLER_DEADBAND ? 0 : curve;
        boolean override = Math.abs(forward) < Config.CURVATURE_OVERRIDE;

        if (buttonPress.get()) {
            if (this.driveBase.getNeutralMode() != NeutralMode.Brake) {
                this.driveBase.setNeutralMode(NeutralMode.Brake);
            }

            this.driveBase.curvatureDrive(forward * forwardSpeedMultiplier, (override ? rotation / breakButtonRotationSpeedDivisor : rotation), override);
        } else {
            if (this.driveBase.getNeutralMode() == NeutralMode.Brake) {
                this.driveBase.setNeutralMode(NeutralMode.Coast);
            }

            this.driveBase.curvatureDrive(forward, (override ? rotation / passiveRotationSpeedDivisor : rotation), override);
        }
    }

    @Override
    public abstract boolean isFinished();

    @Override
    public void end(boolean interrupted) {
        // Go back to disabled mode
      this.driveBase.setDriveMode(DriveBase.DriveMode.Disabled);
    }
}