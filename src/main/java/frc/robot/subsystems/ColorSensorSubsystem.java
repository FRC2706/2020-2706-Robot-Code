/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
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
     * Enum with all the colors we want to detect with the color sensor
     */
    public enum ColorTarget {
        BLUE("Blue", 0.143, 0.427, 0.429),
        GREEN("Green", 0.197, 0.561, 0.240),
        RED("Red", 0.412, 0.398, 0.188),
        YELLOW("Yellow", 0.361, 0.524, 0.113),
        WHITE("White", 0.26, 0.47, 0.26),
        UNKNOWN("Unknown", 0, 0, 0);

        String name;
        Color col;

        ColorTarget(String name, double r, double g, double b) {
            this.col = ColorMatch.makeColor(r, g, b);
            this.name = name;
        }

        public static ColorTarget fromColor(Color col) {
            for (ColorTarget tar : ColorTarget.values()) {
                if (tar.col == col)
                    return tar;
            }
            return ColorTarget.UNKNOWN;
        }
    }

    public class ColorMatchResult {
        public Color rawColor;
        public ColorTarget matchedColor;
        double confidence;
    }

    public ColorSensorSubsystem() {
        // Add the target values to the ColorMatch class
        for (ColorTarget target : ColorTarget.values())
            colorMatch.addColorMatch(target.col);
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
    public ColorMatchResult getClosestColor() {
        ColorMatchResult result = new ColorMatchResult();
        Color c = getColor();
        result.rawColor = c;
        result.matchedColor = ColorTarget.fromColor(colorMatch.matchClosestColor(getColor()).color);
        return result;
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

    @Override
    public void periodic() {
        ColorMatchResult c = getClosestColor();
        SmartDashboard.putString("Detected Color", c.matchedColor.name);
        SmartDashboard.putString("Raw values:", String.format("R: %f, G: %f, B: %f", c.rawColor.red, c.rawColor.green, c.rawColor.blue));
    }
}