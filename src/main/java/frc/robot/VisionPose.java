package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

public class VisionPose {

    private DriveBase drivebase = DriveBaseHolder.getInstance();
    TrajectoryConfig tConfig;

    private Transform2d outerGoalCamera;
    private Transform2d offsetShooter;
    private final double OUTER_GOAL_OFFSET_FROM_WALL = 5.0; // 5m or 16ft from target

    public VisionPose() {
        drivebase = DriveBaseHolder.getInstance();

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Config.ksVolts,
                Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Config.kDriveKinematics, 10);

        tConfig = new TrajectoryConfig(Config.kMaxSpeedMetersPerSecond, Config.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Config.kDriveKinematics).addConstraint(autoVoltageConstraint);

        // Maybe setup a enum for different camera locations
        outerGoalCamera = new Transform2d(new Pose2d(0.2, -0.2, Rotation2d.fromDegrees(0)), new Pose2d());
        offsetShooter = new Transform2d(new Pose2d(), new Pose2d(0, 0.2, Rotation2d.fromDegrees(0)));
    }

    public void initVision(int visionType) {
        switch (visionType) {
        case 0:
            VisionCtrlNetTable.setTapeMode();
        }
    }

    public Pose2d getTargetPose(int visionType) {
        Pose2d relativePose = null;
        switch (visionType) {
        case 0:
            relativePose = OuterGoal();
            break;
        }

        Pose2d fieldPose = transformToField(relativePose);

        return fieldPose;
    }

    private Pose2d OuterGoal() {
        double distanceToTarget = 0; // Get from vision network table
        double robotAngleToTarget = 0; // Get from vision network table
        double perpendicularAngleAtTarget = 0; // Get from vision network table

        if ((int) distanceToTarget == -99 || (int) robotAngleToTarget == -99 || (int) perpendicularAngleAtTarget == -99)
            return null;
        if (distanceToTarget <= 0.5 || distanceToTarget > 6.0)
            return null;
        if (Math.abs(robotAngleToTarget) > 30)
            return null;

        Rotation2d angle = Rotation2d.fromDegrees(robotAngleToTarget);
        double x = distanceToTarget * angle.getCos();
        double y = distanceToTarget * angle.getSin();
        Pose2d relativePose = new Pose2d(x, y, Rotation2d.fromDegrees(perpendicularAngleAtTarget)); 
        // Might need to flip on the sign, +/-, on perpendicularAngleAtTarget

        // Rotation2d angle = Rotation2d.fromDegrees(robotAngleToTarget);
        // Translation2d translation = new Translation2d(distanceToTarget, angle)
        // Pose2d relativePose = translation, Rotation2d.fromDegrees(perpendicularAngleAtTarget)):

        // Offset the the relative pose so the origin is center of robot
        relativePose = relativePose.transformBy(outerGoalCamera);

        // Offset the robot so center of robot drives left of target to align shooter
        relativePose = relativePose.transformBy(offsetShooter);

        // Offset off the wall so robot shoots at a distance
        relativePose = transformOffWall(relativePose, OUTER_GOAL_OFFSET_FROM_WALL);

        return relativePose;
    }

    private Pose2d transformToField(Pose2d relativePose) {
        Transform2d transformToFieldCoordinateSystem = new Transform2d(new Pose2d(), relativePose);
        return drivebase.getPose().transformBy(transformToFieldCoordinateSystem);

    }

    private Pose2d transformOffWall(Pose2d relativePose, double distanceOffWall) {
        Rotation2d rotation = relativePose.getRotation();
        Translation2d offsetTranslation = new Translation2d(rotation.getCos() * distanceOffWall,
                rotation.getSin() * distanceOffWall);
        return relativePose.transformBy(new Transform2d(offsetTranslation, new Rotation2d()));
    }

    /**
     * Generate Ramsete Trajectory from a TargetPose Feed current robot data as
     * startPose
     * 
     * @param targetPose
     * @param tConfig
     * @return
     */
    public Trajectory generateTrajectory(Pose2d targetPose, int visionType) {
        Pose2d currentPose = drivebase.getPose();
        double currentVelocity = drivebase.getAverageSpeed();
        tConfig.setStartVelocity(currentVelocity).setEndVelocity(0).setReversed(getReversed(visionType));

        Trajectory trajectory;
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(currentPose, List.of(), targetPose, tConfig);
        } catch (Exception e) {
            // Put a better log here but if this runs it needs to be logged somewhere, prob
            // include the data that made it fail
            System.out.println("Trajectory using pose from vision data couldn't be generated");
            return null;
        }
        return trajectory;
    }

    private boolean getReversed(int visionType) {
        switch (visionType) {
        case 0:
            return true;
        }

        return false;
    }
}