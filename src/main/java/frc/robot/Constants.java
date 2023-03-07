// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Debug arm position by getting position from Chooser
  public static final boolean armPositionDebugChooser = false;

  // Debug arm position by moving directly there
  public static final boolean armPositionDebugDirect = false;

  public static final double armManualVoltage = 2.0;

  public static final int pigeon2IMU = 0;

  public static final double stickDeadband = 0.1; // TODO We have used 0.05 previously
  public static final double triggerAxisThreshold = 0.10; // Threshold to consider a trigger pulled

  public static final class Swerve {
    public static final int pigeonID = 0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    // Multipliers when speed limit is in effect;
    public static final double speedLimitXY = 0.2; // TODO Raise this for Mecklenberg
    public static final double speedLimitRot = 0.15;

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(19.625);
    public static final double wheelBase = Units.inchesToMeters(23.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
      // To obtain the angleOffset for each angle motor:
      //   - Use a metal bar to align wheels on each side (left/right) straight ahead, with the bevel gears all facing to the right
      //   - Positive input (as supplied by Phoenix tuner) should move the robot forward (or set driveMotorInvert)
      //   - Get the encoder absolute offset from SmartDashboard ("Mod 0 Cancoder", etc.) with robot code enabled
      //   - Set the angleOffset for each module in code
      //   - Build and deploy the updated code
      //   - Power cycle the robot (for good measure)
      //   - Verify results

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(291.09);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(66.80);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(159.60);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(238.27);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final int highMaster = 8;
  public static final int lowerMaster = 9;
  public static final int highSlave = 10;
  public static final int lowerSlave = 11;

  public static final int highCoder = 4; //bob dont delete this comment: future jimmy please document how you did the arm in pheonix tuner x
  public static final int lowCoder = 5;

  // Offset in degrees
  public static final double angleOffsetL = 0;

  // Gear ratio
  public static final double gearRatioL = 450; // 450:1 on the lower arm

  // Constraints for motion profiling
  public static final double velConstraint = 0.5;
  public static final double accelConstraint = 0.5; // setting this to 0.5 for testing purposes

  // Offset in degrees
  public static final double angleOffsetH = 0;

  // Gear ratio
  public static final double gearRatioH = 247.5; // 247.5:1 on the higher arm

  // Constraints for motion profiling
  public static final double velConstraintH = 0.5;
  public static final double accelConstraintH = 0.5; // setting this to 0.5 for testing purposes

  public static final String CANivoreName = "CANivore";

  public static PIDController translationXController = new PIDController(0.50, 0.1, 0); // 10
  public static PIDController translationYController = new PIDController(0.50, 0.1, 0);
  public static PIDController rotationController = new PIDController(0.50, 0.1, 0);

  // public static double maxVelocity = (6380.0 / 60.0 *
  //     SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
  //     SdsModuleConfigurations.MK4I_L2.getWheelDiameter() *
  //     Math.PI);

  public static XboxController dXboxController = new XboxController(0);
  public static XboxController mXboxController = new XboxController(1);

  public static double deadzone = 0.05;

  public static int talonCount = 8;

  public static final double MAX_VOLTAGE = 13.0;

  // public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
  //     * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
  //     * Math.PI;

  public static final double speedLimit = 0.75;
  public static final double rSpeedLimit = 0.40;
  
  public static double translationRateLimit = 2.5;
  public static double rotationRateLimit = 2.5;

  public static double controllerXYExpo = 2.6;
  public static double controllerRoExpo = 2.6;
  // x^3
  // x^1.96 the best so far
  // x^3.4
  // x^2.6
  // x^4.6
  // x^5.4
  // x^0.6

  // public static final int CANDLE = 19;

  // The left-to-right distance between the drivetrain wheels. Should be measured
  // from center to center.
  public static final double swerveWidth = 0.635; // Measure and set trackwidth
  // The front-to-back distance between the drivetrain wheels. Should be measured
  // from center to center.
  public static final double swerveLength = 0.7366; // Measure and set wheelbase
  // public static double maxAngularVelocity = maxVelocity / Math.hypot(swerveWidth / 2.0, swerveLength / 2.0);
  // public static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
  //     maxAngularVelocity, maxAngularVelocity);
}