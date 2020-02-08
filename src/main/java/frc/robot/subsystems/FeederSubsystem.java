/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

/**
 * Add your docs here.
 */
public class FeederSubsystem extends SubsystemBase {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    //FeederSubsystem is a singleton class as it represents a physical subsystem
    private static FeederSubsystem currentInstance;

    //The motor that drives the feeder
    private static WPI_TalonSRX feederTalon;

    //IR sensor that monitors the indexer
    private static AnalogInput indexerIrSensor;

    //How much to shift the feeder wheel when incrementing
    public static FluidConstant<Double> FEEDERSUBSYSTEM_INCREMENT_TICKS = new FluidConstant<>("IncrementTicks", 0.0)
                .registerToTable(Config.constantsTable);
    //Max distance at which the robot knows a ball is at the indexer
    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IR_MAX_DISTANCE = new FluidConstant<>("IrMaxDistance", 0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_P = new FluidConstant<>("FeederSubsystemP", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_I = new FluidConstant<>("FeederSubsystemI", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_D = new FluidConstant<>("FeederSubsystemD", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_F = new FluidConstant<>("FeederSubsystemF", 0.0)
                .registerToTable(Config.constantsTable);
    //Highest speed the motor could reach
    public static FluidConstant<Double> FEEDERSUBSYSTEM_PEAK_OUTPUT = new FluidConstant<>("FeederSubsystemPeakOutput", 0.0) 
                .registerToTable(Config.constantsTable);

    private final int kTimeoutMs = 1000;

    private final int kPIDLoopIdx = 0;

    private FeederSubsystem(){

        //Initialize the talon
        feederTalon = new WPI_TalonSRX(Config.FEEDERSUBSYSTEM_TALON);

        //Initialize the IR sensor
        indexerIrSensor = new AnalogInput(Config.FEEDERSUBSYSTEM_IR_SENSOR);

        //Configure the talon
        feederTalon.configFactoryDefault();
        feederTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
        feederTalon.setSensorPhase(true);
        feederTalon.configNominalOutputForward(0, kTimeoutMs);
        feederTalon.configNominalOutputReverse(0, kTimeoutMs);
        feederTalon.configPeakOutputForward(FEEDERSUBSYSTEM_PEAK_OUTPUT.get(), kTimeoutMs);
        feederTalon.configPeakOutputReverse(-FEEDERSUBSYSTEM_PEAK_OUTPUT.get(), kTimeoutMs);
        feederTalon.configAllowableClosedloopError(0, 0, kTimeoutMs);
        feederTalon.config_kF(kPIDLoopIdx, FEEDERSUBSYSTEM_F.get(), kTimeoutMs);
        feederTalon.config_kP(kPIDLoopIdx, FEEDERSUBSYSTEM_P.get(), kTimeoutMs);
        feederTalon.config_kI(kPIDLoopIdx, FEEDERSUBSYSTEM_I.get(), kTimeoutMs);
        feederTalon.config_kD(kPIDLoopIdx, FEEDERSUBSYSTEM_D.get(), kTimeoutMs);
        int absolutePosition = feederTalon.getSensorCollection().getPulseWidthPosition();
        feederTalon.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
        feederTalon.setSelectedSensorPosition(0, 0, 10);

    }

    public static void init() {
        if (currentInstance == null) {
            currentInstance = new FeederSubsystem();
        }
    }

    public static FeederSubsystem getInstance() {
        init();
        return currentInstance;
    }

    /**
     * Moves the power cells along the feeder track a certain amount
     */
    public void incrementPowerCells(){
        feederTalon.setSelectedSensorPosition(0, 0, 10);
        feederTalon.set(ControlMode.Position, FEEDERSUBSYSTEM_INCREMENT_TICKS.get());
    }

    /**
     * Checks if a power cell has reached the indexer
     * @return whether a power cell has reached the indexer or not
     */
    public boolean isBallAtIndexer(){
        return indexerIrSensor.getVoltage() > FEEDERSUBSYSTEM_IR_MAX_DISTANCE.get();
    }

    /**
     * Runs the feeder motor at a certain speed
     */
    public void runFeeder(){
        feederTalon.set(FEEDERSUBSYSTEM_PEAK_OUTPUT.get());
    }

    /**
     * Runs the feeder motor just enough to empty 5 balls out of the feeder
     */
    public void emptyFeeder(){
        feederTalon.setSelectedSensorPosition(0, 0, 10);
        feederTalon.set(ControlMode.Position, 6*FEEDERSUBSYSTEM_INCREMENT_TICKS.get());
    }

    public void periodic() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}
