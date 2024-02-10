// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
  Orchestra orchestra;

  public Music(Intake intake, Shooter shooter, CommandSwerveDrivetrain drivetrain) {

    orchestra = new Orchestra();
    orchestra.addInstrument(shooter.getFeederMotor());
    orchestra.addInstrument(shooter.getPrimaryWheelMotor());
    orchestra.addInstrument(shooter.getSecondaryWheelMotor());
    orchestra.addInstrument(intake.getIntakeMotor());
    for(TalonFX motor : drivetrain.getMotors()){
      orchestra.addInstrument(motor);
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void playSong(String songName) {
    String deployPath = Filesystem.getDeployDirectory().getAbsolutePath();
    boolean playingMusic = true;
    orchestra.loadMusic(deployPath + "/" + songName + "Output.chrp");
    orchestra.play();
    while(playingMusic) {
      if (!orchestra.isPlaying()) {
        playingMusic = false;
      }
    }
    orchestra.close();
  }
}
