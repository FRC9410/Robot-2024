// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class Subsystems {
    // private Elevator elevator;
    private Shooter shooter;
    private Intake intake;

    public Subsystems() {
        intake = new Intake();
        shooter = new Shooter();
        // elevator = new Elevator();
    }

    public Intake getIntake(){
        return intake;
    }

    public Shooter getShooter(){
        return shooter;
    }

    // public Elevator getElevator(){
    //     return elevator;
    // }
}
