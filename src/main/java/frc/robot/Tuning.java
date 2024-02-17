// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeWrist;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class Tuning {
    private SparkPIDController pidController;
    private double kP= 0;
    private double kI= 0;
    private double kD= 0;
    private double kF= 0;
    private double setpoint;
    // private double maxVel = 2000;
    // private double macAcc = 1500;

    // private static final String intakeWristOption = "Intake wrist";
    // private static final String shooterWristOption = "Shooter wrist";
    // private static final String elevatorOption = "Elevator";
    // private String selected;
    // private final SendableChooser<String> chooser = new SendableChooser<>();

    public Tuning(Subsystems subsystems) {
        this.pidController = this.getPidController(subsystems, "Shooter wrist");
        this.setpoint = 0; //this.getEncoderPosition(subsystems, "Shooter wrist");

        this.pidController.setP(kP);
        this.pidController.setI(kI);
        this.pidController.setD(kD);
        this.pidController.setOutputRange(IntakeWrist.kMinOutput, IntakeWrist.kMaxOutput);

        // chooser.setDefaultOption(intakeWristOption, intakeWristOption);
        // chooser.addOption(shooterWristOption, shooterWristOption);
        // chooser.addOption(elevatorOption, elevatorOption);
        // SmartDashboard.putData("tuner chooser", chooser);

        pidController.setSmartMotionMaxAccel(IntakeWrist.maxAcc, 0);
        pidController.setSmartMotionMaxVelocity(IntakeWrist.maxVel, 0);
        pidController.setSmartMotionAllowedClosedLoopError(IntakeWrist.allowedError, 0);
        
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
        SmartDashboard.putNumber("kF", kF);
        SmartDashboard.putNumber("encoder value", this.getEncoderPosition(subsystems, "Shooter wrist"));
        
    }

    public void updateTuning(Subsystems subsystems) {
        double newSetpoint = SmartDashboard.getNumber("setpoint", 0);
        if (setpoint != newSetpoint) {
        setpoint = newSetpoint;
        pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        }

        double newkP = SmartDashboard.getNumber("kP", kP);
        if (kP != newkP) {
        kP = newkP;
        this.pidController.setP(kP);
        }

        double newkI = SmartDashboard.getNumber("kI", kI);
        if (kI != newkI) {
        kI = newkI;
        this.pidController.setI(kI);
        }

        double newkD = SmartDashboard.getNumber("kD", kD);
        if (kD != newkD) {
        kD = newkD;
        this.pidController.setD(kD);
        }

        double newkF = SmartDashboard.getNumber("kF", kF);
        if (kF != newkF) {
        kF = newkF;
        this.pidController.setD(kF);
        }
    }

    public SparkPIDController getPidController(Subsystems subsystems, String subsystem) {
        switch (subsystem) {
            case "Intake wrist":
                return subsystems.getIntake().getPIDController();
            case "Shooter wrist":
                return subsystems.getShooter().getPIDController();
            default:
                return subsystems.getIntake().getPIDController();
        }
    }

    public double getEncoderPosition(Subsystems subsystems, String subsystem) {
        switch (subsystem) {
            case "Intake wrist":
                return subsystems.getIntake().getEncoderPosition();
            case "Shooter wrist":
                return subsystems.getShooter().getEncoderPosition();
            default:
                return subsystems.getIntake().getEncoderPosition();
        }
    }
}
