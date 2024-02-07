// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterWrist;
import frc.robot.utils.MotorConfig;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase {
  private SparkPIDController pidController;
  private AbsoluteEncoder encoder;

  private static final VelocityVoltage voltageVelocity = new VelocityVoltage(86, 86, true, 0, 0, false, false, false);
    /* Start at velocity 0, no feed forward, use slot 1 */
    private static final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(86, 86, 0, 0, false, false, false);
    /* Keep a neutral out so we can disable the motor */
    private static final NeutralOut brake = new NeutralOut();

  TalonFX feeder = new TalonFX(ShooterWrist.kFeederCanId, RobotConstants.kCtreCanBusName);

  TalonFX primaryWheel = new TalonFX(ShooterWrist.kPrimaryWheelCanId, RobotConstants.kCtreCanBusName);
  TalonFX secondaryWheel = new TalonFX(ShooterWrist.kSecondaryWheelCanId, RobotConstants.kCtreCanBusName);

  CANSparkMax primaryWrist = new CANSparkMax(ShooterWrist.kPrimaryWristCanId, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(ShooterWrist.kSecondaryWristCanId, MotorType.kBrushless);

  private double TuningP = ShooterWrist.kP;
  private double TuningD = ShooterWrist.kD;
  private double TuningFF = ShooterWrist.kFF;
  private double Tuningsetpoint = 0;
  private double TuningMaxOutput = ShooterWrist.kMaxOutput;
  private double TuningMinOutput = ShooterWrist.kMinOutput;


  /** Creates a new Shooter. */
  public Shooter() {
    this.primaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.setInverted(true);
    this.primaryWrist.setIdleMode(IdleMode.kBrake);
    this.secondaryWrist.setIdleMode(IdleMode.kBrake);


    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getAbsoluteEncoder(ShooterWrist.kAbsEncType);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(ShooterWrist.kP);
    this.pidController.setI(ShooterWrist.kI);
    this.pidController.setD(ShooterWrist.kD);
    this.pidController.setIZone(ShooterWrist.kIz);
    this.pidController.setFF(ShooterWrist.kFF);
    this.pidController.setOutputRange(ShooterWrist.kMinOutput, ShooterWrist.kMaxOutput);


    setConfigs(primaryWheel);
    setConfigs(secondaryWheel);

  }

  @Override
  public void periodic() {
    System.out.println(this.primaryWheel.getVelocity());
    // This method will be called once per scheduler run
  }

  public void setShooterVelocity(double velocity) {
   // this.primaryWheel.setVoltage(velocity * 12);
   // this.secondaryWheel.setVoltage(velocity * -0.90 * 12);
   System.out.println(":)");

    this.primaryWheel.setControl(voltageVelocity.withVelocity(-velocity));
    this.secondaryWheel.setControl(voltageVelocity.withVelocity(velocity * 0.95));
  }

  public void shooterOff() {
    this.primaryWheel.setControl(voltageVelocity.withVelocity(0));
    this.secondaryWheel.setControl(voltageVelocity.withVelocity(0));
  }

  public void setWristAngle(double angle) {
    this.pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void wristOff() {
    double currentPosition = encoder.getPosition();
    this.pidController.setReference(currentPosition, CANSparkMax.ControlType.kPosition);
  }

  public void feedOn(double speed) {
    this.feeder.set(speed);
  }

  public void feedOff() {
    this.feeder.set(0);
  }

  public SparkPIDController getPIDController() {
    return this.pidController;
  }

  public double getEncoderPosition() {
    return this.encoder.getPosition();
  }

  public double getFeederPowerDraw() {
    return this.feeder.getSupplyCurrent().getValueAsDouble();
  }

  private static void setConfigs(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0000; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    motor.getConfigurator().apply(configs);
  }

  public void setEnableIdleMode() {
    feeder.setNeutralMode(NeutralModeValue.Brake);
    primaryWheel.setNeutralMode(NeutralModeValue.Brake);
    secondaryWheel.setNeutralMode(NeutralModeValue.Brake);
    primaryWrist.setIdleMode(IdleMode.kBrake);
    secondaryWrist.setIdleMode(IdleMode.kBrake);

  }

  public void setDisableIdleMode() {
    feeder.setNeutralMode(NeutralModeValue.Coast);
    primaryWheel.setNeutralMode(NeutralModeValue.Coast);
    secondaryWheel.setNeutralMode(NeutralModeValue.Coast);
    primaryWrist.setIdleMode(IdleMode.kCoast);
    secondaryWrist.setIdleMode(IdleMode.kCoast);
  }
}
