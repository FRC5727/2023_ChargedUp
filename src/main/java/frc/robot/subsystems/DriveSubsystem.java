package frc.robot.subsystems;


import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.core.json.DupDetector;

import frc.omegabytes.library.omegaSwerveLib.Mk4ModuleConfiguration;
import frc.omegabytes.library.omegaSwerveLib.Mk4iSwerveModuleHelper;
import frc.omegabytes.library.omegaSwerveLib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule flm;
  private final SwerveModule frm;
  private final SwerveModule rlm;
  private final SwerveModule rrm;
  
  private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  private Translation2d offsetPose = new Translation2d(0.0, 0.0);

  private final ShuffleboardTab mTab;

  private static final double maxVoltage = Constants.MAX_VOLTAGE;
  private static final double maxVelocity = Constants.maxVelocity;
  private double currentTranslationPercent;
  private double currentRotationPercent;

  private boolean halfSpeed = false;

  private Pigeon2 pigeon2 = new Pigeon2(Constants.pigeon2IMU, Constants.rickBot);

  public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(Constants.swerveWidth / 2.0, Constants.swerveLength / 2.0),
      // Front right
      new Translation2d(Constants.swerveWidth / 2.0, -Constants.swerveLength / 2.0),
      // Back left
      new Translation2d(-Constants.swerveWidth / 2.0, Constants.swerveLength / 2.0),
      // Back right
      new Translation2d(-Constants.swerveWidth / 2.0, -Constants.swerveLength / 2.0)
  );
  
  public DriveSubsystem() {
    pigeon2.configMountPoseYaw(-90);

    mTab = Shuffleboard.getTab("Drivetrain");
    Mk4ModuleConfiguration CANivore = new Mk4ModuleConfiguration();
    CANivore.setCanivoreName(Constants.rickBot);
    CANivore.useCanivore();
    flm = Mk4iSwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                mTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
                //Initalizes the module onto our CANivore
                CANivore,
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4iSwerveModuleHelper.GearRatio.L2,
                // This is the ID of the drive motor
                Constants.fldmPort,
                // This is the ID of the steer motor
                Constants.flsmPort,
                // This is the ID of the steer encoder
                Constants.flePort,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                Constants.fleo
        ); 
        frm = Mk4iSwerveModuleHelper.createFalcon500(
                mTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
                CANivore,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.frdmPort,
                Constants.frsmPort,
                Constants.frePort,
                Constants.freo
        );
        rlm = Mk4iSwerveModuleHelper.createFalcon500(
                mTab.getLayout("Rear Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
                CANivore,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.rldmPort,
                Constants.rlsmPort,
                Constants.rlePort,
                Constants.rleo
        );
        rrm = Mk4iSwerveModuleHelper.createFalcon500(
                mTab.getLayout("Rear Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
                CANivore,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.rrdmPort,
                Constants.rrsmPort,
                Constants.rrePort,
                Constants.rreo
        );
  }

  public double getGyroPitch(){
    return pigeon2.getPitch();
  }
  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscopeCommand() {
    zeroGyroscope();
  }

  public void zeroGyroscope() {
    pigeon2.zeroGyroBiasNow();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Translation2d getTranslation(){
    return getPose().getTranslation();
  }

  public Pose2d getPose(){
    return new Pose2d(robotPose.getTranslation().minus(offsetPose), robotPose.getRotation());
  }

  public void resetPose(double xValue, double yValue){
    offsetPose = new Translation2d(xValue, yValue);
  }

  public SwerveDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public void resetPose(){
    resetPose(robotPose.getTranslation().getX(), robotPose.getTranslation().getY());
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeon2.getYaw());
  }

  public void toggleHalfSpeed(){
    halfSpeed = !halfSpeed;
  }

  public boolean isHalfSpeed(){
    return halfSpeed;
  }

  public void disableSpeedLimit() {
    currentTranslationPercent = Constants.maxTranslationPercent;
    currentRotationPercent = Constants.maxRotationPercent;
  }

  public void enableSpeedLimit() {
    currentTranslationPercent = Constants.slowTranslationPercent;
    currentTranslationPercent = Constants.slowRotationPercent;
  }

  public double getTranslationPercent() {
    return currentTranslationPercent;
  }

  public double getRotationPercent() {
    return currentRotationPercent;
  }

  public void updateAngle() {}
  public void start(){}

  public void drive(ChassisSpeeds chassisSpeeds) {
    mChassisSpeeds = chassisSpeeds;
  }
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxVelocity);
    
    flm.set(states[0].speedMetersPerSecond / maxVelocity * maxVoltage, states[0].angle.getRadians());
    frm.set(-states[1].speedMetersPerSecond / maxVelocity * maxVoltage, states[1].angle.getRadians()); //Inverted the rear so that it moves correctly
    rlm.set(states[2].speedMetersPerSecond / maxVelocity * maxVoltage, states[2].angle.getRadians());
    rrm.set(-states[3].speedMetersPerSecond / maxVelocity * maxVoltage, states[3].angle.getRadians()); //Inverted the rear so that it moves correctly
  }
  @Override
  public void periodic() {

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(mChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
    SmartDashboard.putNumber("Gyro Angle", pigeon2.getYaw()); //will change to pigeon 2.0 when built

    flm.set(states[0].speedMetersPerSecond / maxVelocity * maxVoltage, states[0].angle.getRadians());
    frm.set(-states[1].speedMetersPerSecond / maxVelocity * maxVoltage, states[1].angle.getRadians()); //Inverted the rear so that it moves correctly
    rlm.set(states[2].speedMetersPerSecond / maxVelocity * maxVoltage, states[2].angle.getRadians());
    rrm.set(-states[3].speedMetersPerSecond / maxVelocity * maxVoltage, states[3].angle.getRadians()); //Inverted the rear so that it moves correctly
    
  }
  public void stop(){
    this.mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(mChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxVelocity);
    flm.set(0.0, Math.toRadians(0.0));
    frm.set(0.0, Math.toRadians(0.0));
    rlm.set(0.0, Math.toRadians(0.0));
    rrm.set(0.0, Math.toRadians(0.0));
  }
}