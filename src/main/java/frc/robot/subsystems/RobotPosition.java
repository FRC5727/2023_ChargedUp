package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.MultiLinearInterpolator;
import frc.robot.Constants;
import frc.robot.Dashboard;

// TODO Vision subsystem, parsing JSON, with timestamp to avoid redundant updates?

public class RobotPosition extends SubsystemBase {
    public static boolean positionDebug = false;
    private boolean useVision = false;
    private boolean lightsOn = false;

    private final MultiLinearInterpolator oneAprilTagLookupTable = new MultiLinearInterpolator(Constants.Vision.ONE_APRIL_TAG_LOOKUP_TABLE);

    private SwerveDrivePoseEstimator swervePose;
    private Swerve s_Swerve;
    private NetworkTable limelightTable;
    private NetworkTableEntry botposeEntry;
    private NetworkTableEntry targetEntry;
    private NetworkTableEntry targetPoseEntry;

    public RobotPosition(Swerve swerve) {
        this.s_Swerve = swerve;

        // The initial pose will be overridden later by the autonomous routine
        swervePose = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, s_Swerve.getYaw(), s_Swerve.getModulePositions(), new Pose2d());

        Dashboard.watchBoolean("Position debug", positionDebug, (val) -> positionDebug = val.booleanValue());
        
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        botposeEntry = limelightTable.getEntry(DriverStation.getAlliance() == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue");
        targetEntry = limelightTable.getEntry("tv");
        targetPoseEntry = limelightTable.getEntry("targetpose_cameraspace");

        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.limelightName);
    }

    public Pose2d getPose() {
        return swervePose.getEstimatedPosition();
    }

    public void resetPosition(Pose2d pose) {
        swervePose.resetPosition(s_Swerve.getYaw(), s_Swerve.getModulePositions(), pose);
    }

    public void disableVision() {
        useVision = false;
        if (lightsOn) {
            LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.limelightName);
            lightsOn = false;
        }
    }

    public void enableVision() {
        useVision = true;
        if (!lightsOn) {
            LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.limelightName);
            lightsOn = true;
        }
    }

    @Override
    public void periodic() {
        Pose2d robotPose = swervePose.update(s_Swerve.getYaw(), s_Swerve.getModulePositions());

        boolean haveTarget = targetEntry.getDouble(0) > 0;
        double[] llpose = botposeEntry.getDoubleArray(new double[7]);
        if (useVision) {
            if (haveTarget && llpose[0] > 0.0 && llpose[1] > 0.0) {
                Pose2d visionPose = new Pose2d(llpose[0], llpose[1], Rotation2d.fromDegrees(llpose[5]));

                if (Math.abs(robotPose.getX() - visionPose.getX()) > Constants.Vision.maxXYError ||
                        Math.abs(robotPose.getY() - visionPose.getY()) > Constants.Vision.maxXYError) {
                    if (DriverStation.isAutonomousEnabled()) {
                        DriverStation.reportWarning("Rejecting vision data with excess error", false);
                    }
                } else {
                    double[] targetPose = targetPoseEntry.getDoubleArray(new double[6]);
                    double targetDistance = Math.sqrt(Math.pow(targetPose[0], 2) + Math.pow(targetPose[1], 2) + Math.pow(targetPose[2], 2));
                    double[] stddev = oneAprilTagLookupTable.getLookupValue(targetDistance);
                    if (positionDebug) {
                        SmartDashboard.putNumber("Target distance", targetDistance);
                    }
                    swervePose.setVisionMeasurementStdDevs(VecBuilder.fill(stddev[0], stddev[1], Units.degreesToRadians(stddev[2])));
                    swervePose.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - (llpose[6] / 1000.0));
                }
            }
        }

        if (lightsOn && !DriverStation.isAutonomousEnabled()) {
            // Don't keep LEDs on past auto
            LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.limelightName);
            lightsOn = false;
        }

        if (positionDebug) {
            // Get position again, in case it's been updated with vision data
            robotPose = swervePose.getEstimatedPosition();
            SmartDashboard.putNumber("Pose X", robotPose.getX());
            SmartDashboard.putNumber("Pose Y", robotPose.getY());
            SmartDashboard.putNumber("Pose Angle", robotPose.getRotation().getDegrees());

            SmartDashboard.putBoolean("Limelight target", haveTarget);
            SmartDashboard.putNumber("Limelight pose X", llpose[0]);
            SmartDashboard.putNumber("Limelight pose Y", llpose[1]);
            SmartDashboard.putNumber("Limelight yaw", llpose[5]);
            SmartDashboard.putNumber("Limelight delay", llpose[6]);
        }
    }
}