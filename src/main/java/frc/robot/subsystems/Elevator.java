// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  CANSparkMax primaryElevator = new CANSparkMax(41, MotorType.kBrushless);
  CANSparkMax secondaryElevator = new CANSparkMax(42, MotorType.kBrushless);


  /** Creates a new Elevator. */
  public Elevator() {
    this.secondaryElevator.follow(primaryElevator);
    this.secondaryElevator.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevator(double speed) {
    primaryElevator.set(speed);
  }
 
  public void elevatorOff() {
    primaryElevator.set(0);
  }
}
