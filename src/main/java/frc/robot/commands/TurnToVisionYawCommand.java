package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;


public class TurnToVisionYawCommand extends CommandBase {

    private Supplier<Double> targetYaw;
    private Double currentTarget;
    private boolean invert;
    private Double maxTime;
    private Double acceptableError;
    private DrivetrainPIDTurnDelta drivetrainPIDTurnDelta;

    public TurnToVisionYawCommand(Supplier<Double> targetYaw, boolean invert) {
        this.targetYaw = targetYaw;
        this.invert = invert;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        currentTarget = targetYaw.get();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // Filter out the default value
        if(currentTarget != -1) {
            // Check if the yaw should be inverted (Shooter is on the back so we may need to)
            if(invert) {
                drivetrainPIDTurnDelta = new DrivetrainPIDTurnDelta(-currentTarget, 0, acceptableError, maxTime);
            } else {
                drivetrainPIDTurnDelta = new DrivetrainPIDTurnDelta(currentTarget, 0, acceptableError, maxTime);
            }
            drivetrainPIDTurnDelta.schedule();
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
        // TODO: Make this return true when this Command no longer needs to run execute()
        return drivetrainPIDTurnDelta.isFinished();
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
