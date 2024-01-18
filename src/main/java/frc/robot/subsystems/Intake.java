// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  private CANSparkMax wristPrimary = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax wristSecondary = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax intake = new CANSparkMax(10, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {
    this.wristSecondary.follow(wristPrimary);
    this.wristSecondary.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    public void wristUp(double speed) {
        wristPrimary.set(speed);
    }
    public void wristDown(double speed) {
        wristPrimary.set(-speed);
    }
     
    public void wristOff() {
      wristPrimary.set(0);
    }
    public void intakeIn(double speed) {
        wristPrimary.set(speed);
    }
    public void intakeOut(double speed) {
        wristPrimary.set(-speed);
    }
     
    public void intakeOff() {
      wristPrimary.set(0);
    }
  }



