/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

/**
 * Add your docs here.
 */
public class FeederSubsystem extends ConditionalSubsystemBase {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    //FeederSubsystem is a singleton class as it represents a physical subsystem
    private static FeederSubsystem currentInstance;

    //The motor that drives the feeder
    private static WPI_TalonSRX feederTalon;

    //IR sensor that monitors the indexer
    private static AnalogInput indexerIrSensor;

    //How much to shift the feeder wheel when incrementing
    public static FluidConstant<Double> FEEDERSUBSYSTEM_INCREMENT_TICKS = new FluidConstant<>("IncrementTicks", 12_000.0)
                .registerToTable(Config.constantsTable);
    //Max distance at which the robot knows a ball is at the indexer
    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IR_MAX_DISTANCE = new FluidConstant<>("IrMaxDistance", 0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_P = new FluidConstant<>("FeederSubsystemP", 0.15)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_I = new FluidConstant<>("FeederSubsystemI", 0.001)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_D = new FluidConstant<>("FeederSubsystemD", 16.5)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_F = new FluidConstant<>("FeederSubsystemF", 0.1)
                .registerToTable(Config.constantsTable);
    //Highest speed the motor could reach
    public static FluidConstant<Double> FEEDERSUBSYSTEM_PEAK_OUTPUT = new FluidConstant<>("FeederSubsystemPeakOutput", 1.0)
                .registerToTable(Config.constantsTable);

    private final int kTimeoutMs = 1000;

    private final int kPIDLoopIdx = 0;

    private FeederSubsystem(){

        createCondition("encoderHealthy", SubsystemConditionStates.ALWAYS);

        //Initialize the talon
        feederTalon = new WPI_TalonSRX(Config.FEEDER_SUBSYSTEM_TALON);

        //Initialize the IR sensor
        // TODO add this once it is in the real robot

        //Configure the talon
        if (checkConditions()){
            feederTalon.configFactoryDefault();
            feederTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
            feederTalon.configNominalOutputForward(0, kTimeoutMs);
            feederTalon.configNominalOutputReverse(0, kTimeoutMs);
            feederTalon.configPeakOutputForward(FEEDERSUBSYSTEM_PEAK_OUTPUT.get(), kTimeoutMs);
            feederTalon.configPeakOutputReverse(-(FEEDERSUBSYSTEM_PEAK_OUTPUT.get()), kTimeoutMs);

           // feederTalon.configAllowableClosedloopError(0, 0, kTimeoutMs);
            feederTalon.config_kF(kPIDLoopIdx, FEEDERSUBSYSTEM_F.get(), kTimeoutMs);
            feederTalon.config_kP(kPIDLoopIdx, FEEDERSUBSYSTEM_P.get(), kTimeoutMs);
            feederTalon.config_kI(kPIDLoopIdx, FEEDERSUBSYSTEM_I.get(), kTimeoutMs);
            feederTalon.config_kD(kPIDLoopIdx, FEEDERSUBSYSTEM_D.get(), kTimeoutMs);
            feederTalon.configAllowableClosedloopError(0, 50, Config.CAN_TIMEOUT_SHORT);
            feederTalon.setSelectedSensorPosition(0, 0, Config.CAN_TIMEOUT_SHORT);

        }

    }

    public static void zeroTalon() {
        feederTalon.getSensorCollection().setQuadraturePosition(0, Config.CAN_TIMEOUT_SHORT);
    }

    public static void init() {
        if (currentInstance == null) {
            currentInstance = new FeederSubsystem();
        }
    }

    public void runAtRPM() {
        double targetVelocity = -1600;
        feederTalon.set(ControlMode.Velocity, targetVelocity);
    }

    public static FeederSubsystem getInstance() {
        init();
        return currentInstance;
    }

    /**
     * Moves the power cells along the feeder track a certain amount
     */
    public void incrementPowerCells(int ticks){
        System.out.println("Incrementing power cells...");
        feederTalon.set(ControlMode.Position, ticks);
    }

    /**
     * Checks if a power cell has reached the indexer
     * @return whether a power cell has reached the indexer or not
     */
    public boolean isBallAtIndexer(){
        return indexerIrSensor.getVoltage() > Config.FEEDERSUBSYSTEM_IR_MAX_DISTANCE.get();
    }

    public void runFeeder(){
     //   System.out.println("Feeder at :" + feederTalon.getSelectedSensorPosition());
        feederTalon.set(ControlMode.PercentOutput, -FEEDERSUBSYSTEM_PEAK_OUTPUT.get());
    }

    public void runFeeder(double speed){
        //   System.out.println("Feeder at :" + feederTalon.getSelectedSensorPosition());
        feederTalon.set(ControlMode.PercentOutput, speed);
    }

    public void slowReverseFeeder() {
        feederTalon.set(ControlMode.PercentOutput, FEEDERSUBSYSTEM_PEAK_OUTPUT.get());
    }

    public void reverseFeeder(){
        feederTalon.set(ControlMode.PercentOutput, -Config.FEEDERSUBSYSTEM_PEAK_OUTPUT.get());
    }

    /**
     * Runs the feeder motor just enough to empty 5 balls out of the feeder
     */
    public void emptyFeeder(){
        feederTalon.setSelectedSensorPosition(0, 0, 10);
        feederTalon.set(ControlMode.Position, 6*Config.FEEDERSUBSYSTEM_INCREMENT_TICKS.get());
    }

    /**
     * Determines if the lift has reached the given setpoint.
     *
     * @return True if the lift has reached the setpoint, false otherwise.
     */
    public boolean doneIncrementing(double lowerLimit) {
        boolean done = false;

        if(feederTalon.getSelectedSensorPosition() <= lowerLimit) {
            done = true;
        }
        return done;
    }

    public int getCurrentPosition() {
        return feederTalon.getSelectedSensorPosition();
    }

    public void periodic() {
        SmartDashboard.putNumber("Feeder encoder ticks", feederTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("Feeder RPM", (feederTalon.getSelectedSensorPosition() * 600.0) / 4096);
    }

    public void stopFeeder() {
        feederTalon.stopMotor();
    }

}