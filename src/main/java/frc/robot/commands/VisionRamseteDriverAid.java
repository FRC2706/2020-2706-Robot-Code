/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionPose;

public class VisionRamseteDriverAid extends CommandBase {

    int visionType;
    VisionPose visionPose;

    final int NUM_POSES_DRIVERAID = 3;
    int currentNumPoses = 0;
    Pose2d[] targetPoses = new Pose2d[NUM_POSES_DRIVERAID];

    /**
     * Creates a new VisionRamseteDriverAid.
     */
    public VisionRamseteDriverAid(int visionType) {
        this.visionType = visionType;

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
        Pose2d targetPose = visionPose.getTargetPose(visionType);
        if (targetPose != null) {
            targetPoses[currentNumPoses] = targetPose;
            currentNumPoses++;
            if (currentNumPoses == NUM_POSES_DRIVERAID) {

                Pose2d averagePose = getAveragePose(targetPoses);
                Trajectory trajectory = visionPose.generateTrajectory(averagePose, visionType);
                RamseteCommandMerge ramseteCommand = new RamseteCommandMerge(trajectory);
                VisionRamseteAuto visionCommand = new VisionRamseteAuto(ramseteCommand, visionType, 0);
                visionCommand.schedule();

                this.cancel(); // <-- may not want to do this. Could maybe make this command triggerWhileHeld
                               // and when this commands ends b/c driver let go of button then cancel the
                               // RamseteCommand, or just always make RamseteCommands cancellable by a specific
                               // button
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private Pose2d getAveragePose(Pose2d[] arr) {
        double x = 0;
        double y = 0;
        double rot = 0;
        for (int i = 0; i < arr.length; i++) {
            x += arr[i].getTranslation().getX();
            y += arr[i].getTranslation().getY();
            rot += arr[i].getRotation().getRadians();
        }
        x /= arr.length;
        y /= arr.length;
        rot /= arr.length;
        return new Pose2d(x, y, new Rotation2d(rot));
    }

}
