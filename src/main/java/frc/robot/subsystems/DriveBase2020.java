package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.config.Config;

import java.util.function.Consumer;

public class DriveBase2020 extends DriveBase {
    WPI_TalonSRX leftMaster, rightMaster;
    WPI_VictorSPX leftSlave, rightSlave;
    
    public DriveBase2020() {
        leftMaster = new WPI_TalonSRX(Config.LEFT_FRONT_TALON);
        rightMaster = new WPI_TalonSRX(Config.RIGHT_FRONT_TALON);
        leftSlave = new WPI_VictorSPX(Config.LEFT_REAR_TALON);
        rightSlave = new WPI_VictorSPX(Config.RIGHT_REAR_TALON);
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
    
        if (Config.PIGEON_ID != -1) {
            pigeon = new PigeonIMU(Config.PIGEON_ID);
            pigeon.setFusedHeading(0d, Config.CAN_TIMEOUT_LONG);
        }
    }
    
    @Override
    public void stopMotors() {
        leftMaster.stopMotor();
        leftSlave.stopMotor();
        rightMaster.stopMotor();
        rightSlave.stopMotor();
    }
    
    @Override
    protected void resetMotors() {
        leftMaster.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightMaster.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        leftSlave.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightSlave.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        
        leftMaster.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
        rightMaster.configPeakCurrentLimit(2, Config.CAN_TIMEOUT_LONG);
    }
    
    @Override
    protected void followMotors() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }
}
