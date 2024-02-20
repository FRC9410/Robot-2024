// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class Utility {

    public static boolean isWithinTolerance(
        double currentValue, double targetValue, double tolerance) {
        return Math.abs(currentValue - targetValue) <= tolerance;
    }

    public static double getSpeed(double speed) {
    double newSpeed = Math.pow(speed, 2);
    return speed > 0 ? newSpeed : -newSpeed;
    // return speed;
    }
}
