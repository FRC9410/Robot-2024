// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

  public NetworkTable shooterTable;
  public NetworkTable intakeTable;

  /** Creates a new Vision. */
  public Vision() {
    shooterTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kShooterTableName);
    shooterTable.getEntry("ledMode").setNumber(1);
    shooterTable.getEntry("pipeline").setNumber(1);
    
    intakeTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kIntakeTableName);
    intakeTable.getEntry("ledMode").setNumber(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTx(VisionType type) {
    return getTable(type).getEntry("tx").getDouble(0);
  }

  public double getTy(VisionType type) {
    return getTable(type).getEntry("ty").getDouble(0);
  }

  public double getTa(VisionType type) {
    return getTable(type).getEntry("ta").getDouble(0);
  }

  public int getTagId(VisionType type) {
    double tagId = getTable(type).getEntry("tid").getDouble(0);
    return (int) tagId;
  }

  public boolean hasTarget(VisionType type) {
    return getTable(type).getEntry("tv").getDouble(0) == 1;
  }

  private NetworkTable getTable(VisionType type) {
    switch (type) {
      case SHOOTER:
        return shooterTable;
      // case INTAKE:
      //   return intakeTable;
      default:
        return null;
    }
  }

  public enum VisionType {
    SHOOTER, INTAKE
  }
}
