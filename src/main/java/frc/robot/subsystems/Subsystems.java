// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class Subsystems {
    // private Elevator elevator;
    private Shooter shooter;
    private Intake intake;
    private Leds leds;
    private Music music;

    public Subsystems() {
        this.intake = new Intake();
        this.shooter = new Shooter();
        this.leds = new Leds();
        this.music = new Music(this.intake, this.shooter);
        // elevator = new Elevator();
    }

    public Intake getIntake() {
        return intake;
    }

    public Shooter getShooter() {
        return shooter;
    }

    // public Elevator getElevator() {
    //     return elevator;
    // }

    public Leds getLeds() {
        return leds;
    }

    public Music getMusic() {
        return music;
    }
}
