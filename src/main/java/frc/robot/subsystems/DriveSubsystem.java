package frc.robot.subsystems;


import com.ctre.phoenix.sensors.Pigeon2;
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
  //private static final double maxAngularVelocity = Constants.maxAngularVelocity;

  private Pigeon2 pigeon2 = new Pigeon2(Constants.pigeon2IMU, Constants.rickBot);

  // private static final TalonFX FLDMTalon = new TalonFX(Constants.fldmPort, Constants.rickBot); //Front Left Drive Motor
  // private static final TalonFX FLSMTalon = new TalonFX(Constants.flsmPort, Constants.rickBot); // Front Left Steer Motor
  // private static final TalonFX FRDMTalon = new TalonFX(Constants.frdmPort, Constants.rickBot); //Front Right Drive Motor
  // private static final TalonFX FRSMTalon = new TalonFX(Constants.frsmPort, Constants.rickBot); // Front Right Steer Motor
  // private static final TalonFX RRDMTalon = new TalonFX(Constants.rrdmPort, Constants.rickBot); //Rear Right Drive Motor
  // private static final TalonFX RRSMTalon = new TalonFX(Constants.rrsmPort, Constants.rickBot); //Rear Right Steer Motor
  // private static final TalonFX RLDMTalon = new TalonFX(Constants.rldmPort, Constants.rickBot); //Rear Left Drive Motor
  // private static final TalonFX RLSMTalon = new TalonFX(Constants.rlsmPort, Constants.rickBot); //Rear Left Steer Motor

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
    // SmartDashboard.putNumber("Pose X", robotPose.getTranslation().getX());
    // SmartDashboard.putNumber("Pose Y", robotPose.getTranslation().getY());
    // SmartDashboard.putNumber("Pose Rotation", robotPose.getRotation().getDegrees());
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

    // String speed = new String(mChassisSpeeds.toString());
    // SmartDashboard.putString("Speed", speed);
    
    // Pose2d poseData = getPose();
    // Field2d field2d = new Field2d();
    // field2d.setRobotPose(poseData);
    // SmartDashboard.putData(field2d);
    
    // SmartDashboard.putNumber("Pose X", poseData.getTranslation().getX());
    // SmartDashboard.putNumber("Pose Y", poseData.getTranslation().getY());
    // SmartDashboard.putNumber("Pose Rotation", poseData.getRotation().getDegrees());
    
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
  public void eBrakeMode(){
    // FLDMTalon.setNeutralMode(NeutralMode.Brake);
    // FLSMTalon.setNeutralMode(NeutralMode.Brake);
    // FRDMTalon.setNeutralMode(NeutralMode.Brake);
    // FRSMTalon.setNeutralMode(NeutralMode.Brake);
    // RRDMTalon.setNeutralMode(NeutralMode.Brake);
    // RRSMTalon.setNeutralMode(NeutralMode.Brake);
    // RLDMTalon.setNeutralMode(NeutralMode.Brake);
    // RLSMTalon.setNeutralMode(NeutralMode.Brake);
  }
  public void unPark(){
    // FLDMTalon.setNeutralMode(NeutralMode.Coast);
    // FLSMTalon.setNeutralMode(NeutralMode.Coast);
    // FRDMTalon.setNeutralMode(NeutralMode.Coast);
    // FRSMTalon.setNeutralMode(NeutralMode.Coast);
    // RRDMTalon.setNeutralMode(NeutralMode.Coast);
    // RRSMTalon.setNeutralMode(NeutralMode.Coast);
    // RLDMTalon.setNeutralMode(NeutralMode.Coast);
    // RLSMTalon.setNeutralMode(NeutralMode.Coast);
  }
}
//https://github.com/BytingBulldogs3539/swerve-lib/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/SdsSwerveModuleHelper.java
//

