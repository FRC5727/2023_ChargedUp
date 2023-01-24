package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsytem. */
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  private ADIS16470_IMU gyro = new ADIS16470_IMU(IMUAxis.kZ, SPI.Port.kOnboardCS0, CalibrationTime._1s);
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  //private final AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP //will change to pigeon 2.0 when built
  
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule mLeftFront;
  private final SwerveModule mRightFront;
  private final SwerveModule mLeftRear;
  private final SwerveModule mRightRear;
  private final ShuffleboardTab mTab;

  private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  //private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0), null); 

  public DriveSubsystem() {
    mTab = Shuffleboard.getTab("Drivetrain");

    mLeftFront = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            mTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            Constants.fldmPort,
            // This is the ID of the steer motor
            Constants.flsmPort,
            // This is the ID of the steer encoder
            Constants.flePort,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            Constants.fleo
    );

    mRightFront = Mk4SwerveModuleHelper.createFalcon500(
        mTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.frdmPort,
            Constants.frsmPort,
            Constants.frePort,
            Constants.freo
    );

    mLeftRear = Mk4SwerveModuleHelper.createFalcon500(
        mTab.getLayout("Rear Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.rldmPort,
            Constants.rlsmPort,
            Constants.rlePort,
            Constants.rleo
    );

    mRightRear = Mk4SwerveModuleHelper.createFalcon500(
        mTab.getLayout("Rear Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
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
  //public CommandBase zeroGyroscopeCommand() {
        //zeroGyroscope();
  //}

  public void zeroGyroscope() {
    gyro.calibrate();
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

  private Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  private Translation2d offsetPose = new Translation2d(0.0, 0.0);

  public void resetPose(double xValue, double yValue){
    offsetPose = new Translation2d(xValue, yValue);
  }

  public void resetPose(){
    resetPose(robotPose.getTranslation().getX(), robotPose.getTranslation().getY());
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void updateAngle() {
    // SmartDashboard.putBoolean("FLM Angle OK", mLeftFront.checkAngle());
    // SmartDashboard.putBoolean("FRM Angle OK", mRightFront.checkAngle());
    // SmartDashboard.putBoolean("RLM Angle OK", mLeftRear.checkAngle());
    // SmartDashboard.putBoolean("RRM Angle OK", mRightRear.checkAngle());
  }
  public void drive(ChassisSpeeds chassisSpeeds) {
    mChassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(mChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle()); //will change to pigeon 2.0 when built
    //m_odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), states);

    mLeftFront.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    mRightFront.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    mLeftRear.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    mRightRear.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}
