/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensorSubsystem extends SubsystemBase {
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch colorMatch = new ColorMatch();

  /**
   * Copied values from RevRobotic's ColorMatch example
   * https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java
   */
  public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public ColorSensorSubsystem() {
    // Add the target values to the ColorMatch class
    colorMatch.addColorMatch(kBlueTarget);
    colorMatch.addColorMatch(kGreenTarget);
    colorMatch.addColorMatch(kRedTarget);
    colorMatch.addColorMatch(kYellowTarget);
  }

  /**
   * Returns the red value of the sensed color
   * 
   * @return Red value
   */
  public int getRed() {
    return colorSensor.getRed();
  }

  /**
   * Returns the blue value of the sensed color
   * 
   * @return Blue value
   */
  public int getBlue() {
    return colorSensor.getBlue();
  }

  /**
   * Returns the green value of the sensed color
   * 
   * @return Green value
   */
  public int getGreen() {
    return colorSensor.getGreen();
  }

  /**
   * Returns the IR value of the sensor
   * 
   * @return IR value
   */
  public int getIR() {
    return colorSensor.getIR();
  }

  /**
   * Returns the sensed proximoty of the sensor
   * 
   * @return Proximity value
   */
  public int getProximity() {
    return colorSensor.getProximity();
  }

  /**
   * Returns a Color object representing the sensed color
   * 
   * @return Color value
   */
  public Color getColor() {
    return colorSensor.getColor();
  }

  /**
   * Returns a raw color value
   * 
   * @return Raw color
   */
  public RawColor getRawColor() {
    return colorSensor.getRawColor();
  }

  /**
   * Sets the confidence threshold for color matching
   */
  public void setColorMatchConfidence(double confidence) {
    colorMatch.setConfidenceThreshold(confidence);
  }

  /**
   * Reads the current color value from the sensor and returns the closest match
   * 
   * @return The exact color value of the closest match
   */
  public Color getClosestColor() {
    return colorMatch.matchClosestColor(getColor()).color;
  }

  /**
   * Reads the current color value from the sensor and returns the closest match
   * 
   * @return The exact color value of the closest match
   */
  public Color getClosestColor(double confidence) {
    colorMatch.setConfidenceThreshold(confidence);
    return colorMatch.matchClosestColor(getColor()).color;
  }
}