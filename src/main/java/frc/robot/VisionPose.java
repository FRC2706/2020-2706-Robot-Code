package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

public class VisionPose {

    private DriveBase drivebase = DriveBaseHolder.getInstance();


    
    public VisionPose() {
        drivebase = DriveBaseHolder.getInstance();
        
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
        Pose2d pose = new Pose2d(); // get data from network tables
        return pose;
    }




    private Pose2d transformToField(Pose2d relativePose) {
        Transform2d transformToFieldCoordinateSystem = new Transform2d(new Pose2d(), relativePose);
        return drivebase.getPose().transformBy(transformToFieldCoordinateSystem);
        
    }

    /**
     * Generate Ramsete Trajectory from a TargetPose
     * Feed current robot data as startPose
     * 
     * @param targetPose
     * @param tConfig
     * @return
     */
    public Trajectory generateTrajectory(Pose2d targetPose, TrajectoryConfig tConfig) {
        Pose2d currentPose = drivebase.getPose();
        double currentVelocity = drivebase.getAverageSpeed();
        tConfig.setStartVelocity(currentVelocity).setEndVelocity(0)
                .setReversed(false); 

        Trajectory trajectory; 
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(currentPose, List.of(), targetPose, tConfig);
        } catch (Exception e) {
            System.out.println("Trajectory using pose from vision data couldn't be generated");
            return null;
        }
        return trajectory;
    }
}