package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Dashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static boolean swerveDebug = false;
    private boolean speedLimit = false;
    
    public SwerveDrivePoseEstimator swervePose;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreName);
        gyro.configFactoryDefault();
        zeroGyro();

        Dashboard.watchBoolean("Swerve debug", swerveDebug, (val) -> swerveDebug = val.booleanValue());

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        // The initial pose will be overridden later by the autonomous routine
        swervePose = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
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
    public void stopDrive() {
        drive(new Translation2d(0, 0), 0, false, true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swervePose.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swervePose.resetPosition(getYaw(), getModulePositions(), pose);
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

    // TODO Remove
    public void hack() {
        gyro.setYaw(gyro.getYaw() + 180);
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

    public double getPitch(){
        // The gyro is rotated 90 degrees, so the gyro roll is the robot pitch
        return gyro.getRoll();
    }

    // TOOD Remove if unused
    public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory traj) {
        SmartDashboard.putString("First pose", traj.getInitialHolonomicPose().toString());
        PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
        SmartDashboard.putString("First pose transformed", transformed.getInitialHolonomicPose().toString());
        return Commands.runOnce(() -> resetOdometry(transformed.getInitialHolonomicPose()))
            .andThen(new PPSwerveControllerCommand(
                traj, 
                this::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(0.5, 0.0, 0.0),
                this::setModuleStates,
                true,
                this));
    }

    @Override
    public void periodic() {
        swervePose.update(getYaw(), getModulePositions());

        if (swerveDebug) {
            for (SwerveModule mod : mSwerveMods) {
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            }
            SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());
            SmartDashboard.putNumber("Robot Pitch", getPitch());
            // SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
            Pose2d robotPose = swervePose.getEstimatedPosition();
            SmartDashboard.putNumber("Pose X", robotPose.getX());
            SmartDashboard.putNumber("Pose Y", robotPose.getY());
            SmartDashboard.putNumber("Pose Angle", robotPose.getRotation().getDegrees());
        }
    }
}
