package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Dashboard;

public class RobotPosition extends SubsystemBase {
    public static boolean positionDebug = false;

    private SwerveDrivePoseEstimator swervePose;
    private Swerve s_Swerve;

    public RobotPosition(Swerve swerve) {
        this.s_Swerve = swerve;

        // The initial pose will be overridden later by the autonomous routine
        swervePose = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, s_Swerve.getYaw(), s_Swerve.getModulePositions(), new Pose2d());

        Dashboard.watchBoolean("Position debug", positionDebug, (val) -> positionDebug = val.booleanValue());
    }

    public Pose2d getPose() {
        return swervePose.getEstimatedPosition();
    }

    public void resetPosition(Pose2d pose) {
        swervePose.resetPosition(s_Swerve.getYaw(), s_Swerve.getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        swervePose.update(s_Swerve.getYaw(), s_Swerve.getModulePositions());

        if (positionDebug) {
            Pose2d robotPose = swervePose.getEstimatedPosition();
            SmartDashboard.putNumber("Pose X", robotPose.getX());
            SmartDashboard.putNumber("Pose Y", robotPose.getY());
            SmartDashboard.putNumber("Pose Angle", robotPose.getRotation().getDegrees());
        }
    }
}