// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private SparkPIDController pidController;
  private RelativeEncoder encoder;

  CANSparkMax primaryElevator = new CANSparkMax(ElevatorConstants.kPrimaryElevatorCanId, MotorType.kBrushless);
  CANSparkMax secondaryElevator = new CANSparkMax(ElevatorConstants.kSecondaryElevatorCanId, MotorType.kBrushless);


  /** Creates a new Elevator. */
  public Elevator() {
    this.secondaryElevator.follow(primaryElevator);
    this.secondaryElevator.setInverted(true);

    this.pidController = primaryElevator.getPIDController();

    this.encoder = primaryElevator.getAlternateEncoder(ElevatorConstants.kAltEncType, ElevatorConstants.kCPR);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(ElevatorConstants.kP);
    this.pidController.setI(ElevatorConstants.kI);
    this.pidController.setD(ElevatorConstants.kD);
    this.pidController.setIZone(ElevatorConstants.kIz);
    this.pidController.setFF(ElevatorConstants.kFF);
    this.pidController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
  }

  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorPosition(double distance) {
    pidController.setReference(distance, CANSparkMax.ControlType.kPosition);
  }
  //might change later to set distance :)
 
  public void elevatorOff() {
    double currentPosition = encoder.getPosition();
    pidController.setReference(currentPosition, CANSparkMax.ControlType.kPosition);
  }
}
