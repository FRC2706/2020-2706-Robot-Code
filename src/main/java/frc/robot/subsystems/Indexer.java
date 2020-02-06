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
public class Indexer extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

    private static Indexer currentInstance;

    private static WPI_TalonSRX indexerTalon;

    private static AnalogInput indexerIrSensor;

    public static FluidConstant<Double> INCREMENT_TICKS = new FluidConstant<>("IncrementTicks", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Integer> IR_MIN_DISTANCE = new FluidConstant<>("IrMinDistance", 0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> INDEXER_P = new FluidConstant<>("IndexerP", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> INDEXER_I = new FluidConstant<>("IndexerI", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> INDEXER_D = new FluidConstant<>("IndexerD", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> INDEXER_F = new FluidConstant<>("IndexerF", 0.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> INDEXER_PEAK_OUTPUT = new FluidConstant<>("IndexerPeakOutput", 0.0) 
                .registerToTable(Config.constantsTable);

    private final int kTimeoutMs = 1000;

    private final int kPIDLoopIdx = 0;

    private Indexer(){

        indexerTalon = new WPI_TalonSRX(Config.INDEXER_TALON);
        indexerIrSensor = new AnalogInput(Config.INDEXER_IR_SENSOR);

        indexerTalon.configFactoryDefault();
        indexerTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
        indexerTalon.setSensorPhase(true);
        indexerTalon.configNominalOutputForward(0, kTimeoutMs);
        indexerTalon.configNominalOutputReverse(0, kTimeoutMs);
        indexerTalon.configPeakOutputForward(INDEXER_PEAK_OUTPUT.get(), kTimeoutMs);
        indexerTalon.configPeakOutputReverse(-INDEXER_PEAK_OUTPUT.get(), kTimeoutMs);
        indexerTalon.configAllowableClosedloopError(0, 0, kTimeoutMs);
        indexerTalon.config_kF(kPIDLoopIdx, INDEXER_F.get(), kTimeoutMs);
        indexerTalon.config_kP(kPIDLoopIdx, INDEXER_P.get(), kTimeoutMs);
        indexerTalon.config_kI(kPIDLoopIdx, INDEXER_I.get(), kTimeoutMs);
        indexerTalon.config_kD(kPIDLoopIdx, INDEXER_D.get(), kTimeoutMs);
        int absolutePosition = indexerTalon.getSensorCollection().getPulseWidthPosition();
        indexerTalon.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
        indexerTalon.setSelectedSensorPosition(0, 0, 10);

    }

    public static void init() {
        if (currentInstance == null) {
            currentInstance = new Indexer();
        }
    }

    public static Indexer getInstance() {
        init();
        return currentInstance;
    }

    public void incrementPowerCells(){
        indexerTalon.setSelectedSensorPosition(0, 0, 10);
        indexerTalon.set(ControlMode.Position, INCREMENT_TICKS.get());
    }

    public boolean isBallAtIndexer(){
        return indexerIrSensor.getVoltage() > IR_MIN_DISTANCE.get();
    }

    public void emptyIndexer(){
        indexerTalon.setSelectedSensorPosition(0, 0, 10);
        indexerTalon.set(ControlMode.Position, 6*INCREMENT_TICKS.get());
    }

    public void periodic() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}
