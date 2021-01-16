/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;

import edu.wpi.first.wpilibj.geometry.Pose2d;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This command was copied and modified from RamseteCommand on WpiLib.
 * https://github.com/wpilibsuite/allwpilib/blob/master/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/RamseteCommand.java
 * 
 * A command that uses a RAMSETE controller ({@link RamseteController}) to
 * follow a trajectory {@link Trajectory} with a differential drive.
 *
 * <p>
 * The command handles trajectory-following, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by
 * teams without a great deal of controls expertise.
 *
 * <p>
 * Advanced teams seeking more flexibility (for example, those who wish to use
 * the onboard PID functionality of a "smart" motor controller) may use the
 * secondary constructor that omits the PID returning only the raw wheel speeds
 * from the RAMSETE controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommandMerge extends CommandBase {
    private final Timer m_timer = new Timer();
    private Trajectory m_trajectory;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_totalTimeElapsed = 0;
    private double m_timeBeforeTrajectory = 0;
    private double m_prevTime;
    private final DriveBase m_driveSubsystem;
    private Pose2d targetPose;

    // private NetworkTableEntry xError, yError, rotError;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided
     * trajectory. Performs P control and calculates feedforwards; outputs are the
     * raw wheel speeds from the RAMSETE controller and the feedforwards. It will
     * follow the full trajectory
     *
     * @param trajectory The trajectory to follow.
     */
    public RamseteCommandMerge(Trajectory trajectory) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommandMerge");

        m_driveSubsystem = DriveBaseHolder.getInstance();

        m_follower = new RamseteController(Config.kRamseteB, Config.kRamseteZeta);
        m_kinematics = Config.kDriveKinematics;

        m_feedforward = new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter,
                Config.kaVoltSecondsSquaredPerMeter);

        addRequirements(m_driveSubsystem);

        // var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        // xError = table.getEntry("xError");
        // yError = table.getEntry("yError");
        // rotError = table.getEntry("rotError");
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        targetPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;
    }

    @Override
    public void execute() {
        m_totalTimeElapsed = m_timer.get();
        double curTime = m_totalTimeElapsed - m_timeBeforeTrajectory;
        double dt = m_totalTimeElapsed - m_prevTime;

        Pose2d currentPose = m_driveSubsystem.getPose();
        Trajectory.State desiredState = m_trajectory.sample(curTime);

        // Network Table stuff
        // Pose2d poseError = desiredState.poseMeters.relativeTo(currentPose);
        // xError.setNumber(poseError.getTranslation().getX());
        // yError.setNumber(poseError.getTranslation().getY());
        // rotError.setNumber(poseError.getRotation().getDegrees());

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(m_follower.calculate(currentPose, desiredState));

        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftAcceleration = (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt;
        double rightAcceleration = (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt;

        double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint, leftAcceleration);
        double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint, rightAcceleration);

        // m_driveSubsystem.logRamseteData(currentPose, desiredState, poseError,
        // leftSpeedSetpoint, rightSpeedSetpoint,
        // leftFeedforward, rightFeedforward, leftAcceleration, rightAcceleration,
        // targetPose);

        m_driveSubsystem.tankDriveVelocities(leftSpeedSetpoint, rightSpeedSetpoint, leftFeedforward, rightFeedforward);

        m_prevTime = m_totalTimeElapsed;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        // m_driveSubsystem.stopLogging();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds() + m_timeBeforeTrajectory);
    }

    public double getTotalTime() {
            return m_trajectory.getTotalTimeSeconds();
    }

    public double getElapsedTime() {
        return m_totalTimeElapsed;
    }

    public Trajectory.State getFutureState(double timeInFuture) {
        return m_trajectory.sample(m_totalTimeElapsed - m_timeBeforeTrajectory + timeInFuture);
    }

    public void setNewTrajectory(Trajectory newTrajectory) {
        m_trajectory = newTrajectory;
        m_timeBeforeTrajectory = m_timer.get();
        targetPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;
    }

}