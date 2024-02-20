package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final PIDController rotationPidController =
        new PIDController(DriveConstants.rotationKP, DriveConstants.rotationkI, DriveConstants.rotationkD);

    private final SwerveRequest.ApplyChassisSpeeds chassisSpeedRequest =
        new SwerveRequest.ApplyChassisSpeeds();

    // Comment out below requests for CUBE_BOT
    private final SwerveRequest.FieldCentric fieldRelative =
        new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * OIConstants.LEFT_X_DEADBAND)
            .withRotationalDeadband(DriveConstants.MaxAngularRate * OIConstants.RIGHT_X_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotRelative =
        new SwerveRequest.RobotCentric()
            .withDeadband(DriveConstants.MaxSpeed * OIConstants.LEFT_X_DEADBAND)
            .withRotationalDeadband(DriveConstants.MaxAngularRate * OIConstants.RIGHT_X_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants driveTrainConstants,
        double OdometryUpdateFrequency,
        SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0, modules);
    }

    public void applyRequest(Supplier<SwerveRequest> requestSupplier) {
    setControl(requestSupplier.get());
    }

    public void setYaw(double yaw) {
    getPigeon2().setYaw(yaw);
    }

    public void drive(
        double velocityXMetersPerSecond,
        double velocityYMetersPerSecond,
        double rotationRateRadiansPerSecond,
        DriveMode mode) {

        switch (mode) {
            case ROBOT_RELATIVE:
            applyRequest(
                () ->
                    robotRelative
                        .withVelocityX(-velocityXMetersPerSecond)
                        .withVelocityY(-velocityYMetersPerSecond)
                        .withRotationalRate(-rotationRateRadiansPerSecond));
            break;
            case FIELD_RELATIVE:
            applyRequest(
                () ->
                    fieldRelative
                        .withVelocityX(-velocityXMetersPerSecond)
                        .withVelocityY(-velocityYMetersPerSecond)
                        .withRotationalRate(-rotationRateRadiansPerSecond));
            break;
            case TARGET_LOCK:
            applyRequest(
                () ->
                    fieldRelative
                        .withVelocityX(-velocityXMetersPerSecond)
                        .withVelocityY(-velocityYMetersPerSecond)
                        .withRotationalRate(getTargetLockRotation(rotationRateRadiansPerSecond)));
            break;
        }
    }

    /**
     * @return A list of the module positions in the order Front Left, Front Right, Back Left, Back
     *     Right
     */
    public SwerveModulePosition[] getModulePositions() {
    return super.m_modulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
    return super.getState().ModuleStates;
    }

    public SwerveModuleState[] getModuleTargets() {
    return super.getState().ModuleTargets;
    }

    public Pose2d getPose() {
    return super.m_odometry.getEstimatedPosition();
    }

    public Supplier<Pose2d> getPoseSupplier() {
    return new Supplier<Pose2d>() {

        @Override
        public Pose2d get() {
        return getPose();
        }
    };
    }

    private SwerveDriveKinematics getKinematics() {
        return super.m_kinematics;
    }

    public void zeroAll() {
        zeroGyro();
        resetPose();
    }

    public void zeroGyro() {
        super.getPigeon2().setYaw(0);
    }

    public void resetPose() {
        setPose(new Pose2d(0, 0, new Rotation2d()));
    }

    public void stopMotorsIntoX() {
        applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }

    public void stopMotors() {
        drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
    }

    public void setPose(Pose2d poseToSet) {
        super.seedFieldRelative(poseToSet);
    }

    public Consumer<ChassisSpeeds> getChassisSpeedsConsumer() {
        return new Consumer<ChassisSpeeds>() {
            @Override
            public void accept(ChassisSpeeds speeds) {
            SwerveModuleState[] moduleStates = getKinematics().toSwerveModuleStates(speeds);
            for (SwerveModuleState state : moduleStates) {
                state.speedMetersPerSecond += DriveConstants.staticKFF;
            }
            setControl(chassisSpeedRequest.withSpeeds(getKinematics().toChassisSpeeds(moduleStates)));
            }
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = getKinematics().toChassisSpeeds(getModuleStates());
    return chassisSpeeds;
    }

    public double getTargetLockRotation(double tx) {
        // Increase kP based on horizontal velocity to reduce lag
        double vy = getChassisSpeeds().vyMetersPerSecond; // Horizontal velocity
        double kp = DriveConstants.rotationKP;
        kp *= Math.max(1, vy * 1);
        rotationPidController.setP(kp);

        double rotation = -rotationPidController.calculate(0, tx);
        double output = rotation + Math.copySign(DriveConstants.targetLockKFF, rotation);
        return output;
    }

    public TalonFX[] getMotors() {
        TalonFX[] motors = new TalonFX[8];
        motors[0] = this.Modules[0].getDriveMotor();
        motors[1] = this.Modules[0].getSteerMotor();
        motors[2] = this.Modules[1].getDriveMotor();
        motors[3] = this.Modules[1].getSteerMotor();
        motors[4] = this.Modules[2].getDriveMotor();
        motors[5] = this.Modules[2].getSteerMotor();
        motors[6] = this.Modules[3].getDriveMotor();
        motors[7] = this.Modules[3].getSteerMotor();


        return motors;
    }

    public void configDriveMotors() {
        TalonFX[] motors = this.getMotors();
        TalonFXConfiguration config = new TalonFXConfiguration();
        OpenLoopRampsConfigs openLoopRamps = new OpenLoopRampsConfigs();
        openLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
        config.OpenLoopRamps = openLoopRamps;
        
        for(TalonFX motor : motors){
            motor.setNeutralMode(NeutralModeValue.Coast);
            motor.getConfigurator().apply(config);
        }
    }

    public enum DriveMode {
    ROBOT_RELATIVE, FIELD_RELATIVE, TARGET_LOCK
    }
}
