package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;

import java.util.logging.Level;
import java.util.logging.Logger;


public class TurnToOuterPortCommand extends CommandBase {

    // The target angle to turn the robot to
    private Double currentTarget;

    // Weather or not too invert the direction turned (true if aiming the back of the robot)
    private boolean invert;

    // The maximum amount of time the command is allowed to run
    private Double maxTime;

    // The acceptable error for the target angle
    private Double acceptableError;

    // Logging
    private Logger logger = Logger.getLogger("TurnOuterPort");

    public TurnToOuterPortCommand(boolean invert, Double acceptableError, Double maxTime) {
        this.invert = invert;
        this.maxTime = maxTime;
        this.acceptableError = acceptableError;

        logger.addHandler(Config.logFileHandler);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // Ensure the vision is running in tape mode
        VisionCtrlNetTable.setTapeMode();

        // Get the target angle from NetworkTables
        currentTarget = VisionCtrlNetTable.yawToOuterPort.get();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // Filter out the default value
        DrivetrainPIDTurnDelta drivetrainPIDTurnDelta;
        if (currentTarget != -99) {
            // Filter out outlying values
            if(currentTarget <= 30 && currentTarget >= -30) {
                // Check if the yaw should be inverted (Shooter is on the back so we may need to)
                if (invert) {
                    drivetrainPIDTurnDelta = new DrivetrainPIDTurnDelta(-currentTarget, 0, acceptableError, maxTime);
                } else {
                    drivetrainPIDTurnDelta = new DrivetrainPIDTurnDelta(currentTarget, 0, acceptableError, maxTime);
                }
                drivetrainPIDTurnDelta.schedule();
            }
            else {
                // Ensure no movement on faulty values
                drivetrainPIDTurnDelta = new DrivetrainPIDTurnDelta(0, 0, 0d, 0d);
                logger.log(Level.WARNING, "Invalid Current Target: " + VisionCtrlNetTable.yawToOuterPort.get());
            }
        } else {
            // Ensure no movement on faulty values
            drivetrainPIDTurnDelta = new DrivetrainPIDTurnDelta(0, 0, 0d, 0d);
            logger.log(Level.WARNING, "Invalid Current Target (Value Not Read): " + VisionCtrlNetTable.yawToOuterPort.get());
        }

    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // This command should only be run once
        return true;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
