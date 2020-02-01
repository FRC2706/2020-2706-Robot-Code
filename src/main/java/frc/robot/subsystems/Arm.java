package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    // TODO Change placeholder value to robotSpecific
    WPI_TalonSRX armTalon;

    private Arm() {
        // Init all the talon values
       armTalon = new WPI_TalonSRX(-1);


    }

}
