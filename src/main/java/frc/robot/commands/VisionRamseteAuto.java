/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionPose;
import frc.robot.subsystems.DriveBase;

public class VisionRamseteAuto extends CommandBase {

    DriveBase drivebase;
    VisionPose visionPose;

    RamseteCommandMerge ramseteCommand;

    int visionType;
    double visionStartTime;

    Pose2d prevPose;
    int cooldown;
    final int COOLDOWN_CYCLES = 6;

    /**
     * Creates a new VisionRamsete.
     */
    public VisionRamseteAuto(RamseteCommandMerge command, int visionType, double startVisionAfterTimeElasped) {
        ramseteCommand = command;
        this.visionType = visionType;
        visionStartTime = startVisionAfterTimeElasped;

        prevPose = ramseteCommand.getTargetPose();

        visionPose = new VisionPose(); // <- make a singleton class?
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        visionPose.initVision(visionType);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ramseteCommand.getElapsedTime() > visionStartTime) {
            Pose2d targetPose = averageTargetPose(visionPose.getTargetPose(visionType));
            if (targetPose != null) {
                if (shouldGenerateTrajectory() && prevPose != targetPose) {
                    Trajectory trajectory = visionPose.generateTrajectory(targetPose, visionType);
                    if (trajectory != null) {
                        ramseteCommand.setNewTrajectory(trajectory);
                        cooldown = 0;
                        prevPose = targetPose;
                    }
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            ramseteCommand.cancel();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }

    private boolean shouldGenerateTrajectory() {
        if (cooldown < COOLDOWN_CYCLES) {
            cooldown++;
            return false;
        }

        return true;
    }

    private Pose2d averageTargetPose(Pose2d pose) {
        return pose;

        // take the average of all the poses generated from vision.
        //
        // todo: put some intelligence behind the average, like a
        // weighted average based on distance or age, remove outliers

    }

}
