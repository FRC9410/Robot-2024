// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterWrist;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeWrist;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.LinearInterpolator;

public class Shooter extends SubsystemBase {
  private SparkPIDController pidController;
  private RelativeEncoder encoder;

  private LinearInterpolator shooterVelocityInterpolator;
  private LinearInterpolator feederVelocityInterpolator;
  private LinearInterpolator wristAngleInterpolator;

  private static final VelocityVoltage voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  private static final VelocityVoltage voltageVelocityFoc = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private static final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(86, 86, 0, 0, false, false, false);
  private static final NeutralOut brake = new NeutralOut();

  private TalonFX feeder = new TalonFX(ShooterWrist.kFeederCanId, RobotConstants.kCtreCanBusName);

  private TalonFX primaryWheel = new TalonFX(ShooterWrist.kPrimaryWheelCanId, RobotConstants.kCtreCanBusName);
  private TalonFX secondaryWheel = new TalonFX(ShooterWrist.kSecondaryWheelCanId, RobotConstants.kCtreCanBusName);

  CANSparkMax primaryWrist = new CANSparkMax(ShooterWrist.kPrimaryWristCanId, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(ShooterWrist.kSecondaryWristCanId, MotorType.kBrushless);

  private double TuningP = ShooterWrist.kP;
  private double TuningD = ShooterWrist.kD;
  private double TuningFF = ShooterWrist.kFF;
  private double Tuningsetpoint = 0;
  private double TuningMaxOutput = ShooterWrist.kMaxOutput;
  private double TuningMinOutput = ShooterWrist.kMinOutput;

  private double wristAngle = 0;

  private double setpoint;
  
  private double shooterSetpoint;
  private double feederSetpoint;
  private double wristSetpoint;

  /** Creates a new Shooter. */
  public Shooter() {
    this.shooterVelocityInterpolator = new LinearInterpolator(ShooterConstants.shooterSpeeds);
    this.feederVelocityInterpolator = new LinearInterpolator(ShooterConstants.feederSpeeds);
    this.wristAngleInterpolator = new LinearInterpolator(ShooterWrist.wristAngles);
    this.primaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.setInverted(true);
    this.primaryWrist.setIdleMode(IdleMode.kBrake);
    this.secondaryWrist.setIdleMode(IdleMode.kBrake);


    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getEncoder();
    this.encoder.setPosition(-0.2);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(ShooterWrist.kP);
    this.pidController.setI(ShooterWrist.kI);
    this.pidController.setD(ShooterWrist.kD);
    this.pidController.setIZone(ShooterWrist.kIz);
    this.pidController.setFF(ShooterWrist.kFF);
    this.pidController.setOutputRange(ShooterWrist.kMinOutput, ShooterWrist.kMaxOutput);
    
    pidController.setSmartMotionMaxAccel(1000, 0);
    pidController.setSmartMotionMaxVelocity(IntakeWrist.maxVel, 0);
    pidController.setSmartMotionAllowedClosedLoopError(IntakeWrist.allowedError, 0);
    
    this.setpoint = ShooterWrist.kMinRotation;
    this.pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("shooter setpoint", setpoint);
    // This method will be called once per scheduler run


    // setShooterConfigs(primaryWheel);
    // setShooterConfigs(secondaryWheel);
    setFeederConfigs(feeder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(this.encoder.getPosition());
    
    double newSetpoint = SmartDashboard.getNumber("shooter setpoint", IntakeWrist.kMinRotation);
    if (setpoint != newSetpoint) {
      setpoint = newSetpoint;
      this.pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    SmartDashboard.putNumber("Shooter value", this.encoder.getPosition());
    System.out.println(primaryWheel.getVelocity());
  }

  public void setShooterVelocity(double velocity) {
    this.primaryWheel.setControl(voltageVelocity.withVelocity(-velocity).withFeedForward(-ShooterConstants.kFF)); //-100
    this.secondaryWheel.setControl(voltageVelocity.withVelocity(velocity-5).withFeedForward(ShooterConstants.kFF)); //95
  }

  public void setShooterVelocity() {
    this.primaryWheel.setControl(voltageVelocity.withVelocity(-shooterSetpoint).withFeedForward(-ShooterConstants.kFF)); //-100
    this.secondaryWheel.setControl(voltageVelocity.withVelocity(shooterSetpoint-5).withFeedForward(ShooterConstants.kFF)); //95
  }

  public void shooterOff() {
    this.primaryWheel.setControl(brake);
    this.secondaryWheel.setControl(brake);
  }

  public void setWristAngle(double angle) {
    if ((wristAngle + angle) >= 0)
    wristAngle += angle;
    this.pidController.setReference(wristAngle, CANSparkMax.ControlType.kPosition);
  }

  public void setWristAngle() {
    this.pidController.setReference(wristSetpoint, CANSparkMax.ControlType.kPosition);
  }

  public void wristOff() {
    double currentPosition = encoder.getPosition();
    this.pidController.setReference(currentPosition, CANSparkMax.ControlType.kPosition);
  }

  public void feedOn(double velocity, double feedforward) {
    this.feeder.setControl(voltageVelocity.withVelocity(80));
  }

  public void feedOff() {
    this.feeder.setControl(brake);
  }

  public void setFeederVelocity(double velocity) {
    this.feeder.setControl(voltageVelocityFoc.withVelocity(velocity));

  }

  public void setFeederVelocity() {
    this.feeder.setControl(voltageVelocityFoc.withVelocity(feederSetpoint));
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

  public TalonFX getFeederMotor() {
    return this.feeder;
  }

  public TalonFX getPrimaryWheelMotor() {
    return this.primaryWheel;
  }

  public TalonFX getSecondaryWheelMotor() {
    return this.secondaryWheel;
  }

  private static void setShooterConfigs(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.0; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 16;
    configs.Voltage.PeakReverseVoltage = 
    -16;

    motor.getConfigurator().apply(configs);
  }

  private static void setFeederConfigs(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.3; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0000; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    motor.getConfigurator().apply(configs);
  }

  public void setShooterVelocitySetpoint(double ty){
    shooterSetpoint = shooterVelocityInterpolator.getInterpolatedValue(ty);
  }

  public void setFeederVelocitySetpoint(double ty){
    feederSetpoint = feederVelocityInterpolator.getInterpolatedValue(ty);
  }

  public void setWristAngleSetpoint(double ty){
    wristSetpoint = wristAngleInterpolator.getInterpolatedValue(ty);
  }

  public void setEnableIdleMode() {
    feeder.setNeutralMode(NeutralModeValue.Coast);
    primaryWheel.setNeutralMode(NeutralModeValue.Coast);

    secondaryWheel.setNeutralMode(NeutralModeValue.Coast);
    primaryWrist.setIdleMode(IdleMode.kCoast);
    secondaryWrist.setIdleMode(IdleMode.kCoast);

  }

  public void setDisableIdleMode() {
    feeder.setNeutralMode(NeutralModeValue.Coast);
    primaryWheel.setNeutralMode(NeutralModeValue.Coast);
    secondaryWheel.setNeutralMode(NeutralModeValue.Coast);
    primaryWrist.setIdleMode(IdleMode.kBrake);
    secondaryWrist.setIdleMode(IdleMode.kBrake);
  }
}
