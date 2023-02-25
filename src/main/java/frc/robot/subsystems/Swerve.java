package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private boolean swerveDebug = false;
    private boolean speedLimit = false;
    
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private Pose2d robotPose = new Pose2d(0, 0.0, Rotation2d.fromDegrees(0.0));
    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreName);
        gyro.configFactoryDefault();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, new SwerveModuleConstants(
                Constants.Swerve.Mod0.driveMotorID,
                Constants.Swerve.Mod0.angleMotorID,
                Constants.Swerve.Mod0.canCoderID,
                Rotation2d.fromDegrees(Robot.practice ? Constants.Swerve.practiceOffsets[0] : Constants.Swerve.compOffsets[0]))),
            new SwerveModule(1, new SwerveModuleConstants(
                Constants.Swerve.Mod1.driveMotorID,
                Constants.Swerve.Mod1.angleMotorID,
                Constants.Swerve.Mod1.canCoderID,
                Rotation2d.fromDegrees(Robot.practice ? Constants.Swerve.practiceOffsets[1] : Constants.Swerve.compOffsets[1]))),
            new SwerveModule(2, new SwerveModuleConstants(
                Constants.Swerve.Mod2.driveMotorID,
                Constants.Swerve.Mod2.angleMotorID,
                Constants.Swerve.Mod2.canCoderID,
                Rotation2d.fromDegrees(Robot.practice ? Constants.Swerve.practiceOffsets[2] : Constants.Swerve.compOffsets[2]))),
            new SwerveModule(3, new SwerveModuleConstants(
                Constants.Swerve.Mod3.driveMotorID,
                Constants.Swerve.Mod3.angleMotorID,
                Constants.Swerve.Mod3.canCoderID,
                Rotation2d.fromDegrees(Robot.practice ? Constants.Swerve.practiceOffsets[3] : Constants.Swerve.compOffsets[3])))
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        gyro.addYaw(180); // TODO Should this depending on the starting orientation?
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void enableSpeedLimit() {
        speedLimit = true;
    }

    public void disableSpeedLimit() {
        speedLimit = false;
    }

    public double getSpeedLimitXY() {
        return speedLimit ? Constants.Swerve.speedLimitXY : 1.0;
    }

    public double getSpeedLimitRot() {
        return speedLimit ? Constants.Swerve.speedLimitRot : 1.0;
    }

    public void stop() {
        // TODO Add convenience code to stop the robot -- see old DriveSubsystem::stop()
        // for example
    }

    public double getGyroPitch(){
        return gyro.getPitch();
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());

        if (swerveDebug) {
            for (SwerveModule mod : mSwerveMods) {
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                        mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            }
            SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());
            robotPose = swerveOdometry.getPoseMeters();
            SmartDashboard.putNumber("Pose X", robotPose.getX());
            SmartDashboard.putNumber("Pose Y", robotPose.getY());
        }
    }
}