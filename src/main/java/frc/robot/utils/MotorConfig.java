// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class MotorConfig {
    /* Be able to switch which control request to use based on a button press */
    /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
    private static final VelocityVoltage voltageVelocity = new VelocityVoltage(10000, 10000, true, 0, 0, false, false, false);
    /* Start at velocity 0, no feed forward, use slot 1 */
    private static final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
    /* Keep a neutral out so we can disable the motor */
    private static final NeutralOut brake = new NeutralOut();


    public static VelocityVoltage getVoltageVelocity() {
        return voltageVelocity;
    }

    public static VelocityTorqueCurrentFOC getTorqueVelocity() {
        return torqueVelocity;
    }

    public static NeutralOut getBrake() {
        return brake;
    }

    

    
}
