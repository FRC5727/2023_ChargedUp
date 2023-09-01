// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.math.BetterSwerveKinematics;
import frc.lib.math.SecondOrderSwerveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String CANivoreName = "CANivore";

  public static final int LED_CANDLE = 0; // Port for LED light

  public static final class Controls {
    public static final double stickDeadband = 0.10; // TODO We have used 0.05 previously
    public static final double triggerAxisThreshold = 0.10; // Threshold to consider a trigger pulled
    public static final XboxController driver = new XboxController(0);
  }

  public static final class Swerve {
    public static final int pigeonID = 0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    // Multipliers when speed limit is in effect;
    public static final double speedLimitXY = 0.45;
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
      
    public static final SecondOrderSwerveKinematics secondKinematics = new SecondOrderSwerveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final BetterSwerveKinematics betterKinematics = new BetterSwerveKinematics(
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
    public static final double maxAngularVelocity = Units.degreesToRadians(450); // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
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
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(292.939453);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(199.160156);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(157.851563);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(237.041016);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class Arm {
    // CAN bus IDs
    public static final int upperMaster = 8;
    public static final int lowerMaster = 9;
    public static final int upperSlave = 10;
    public static final int lowerSlave = 11;

    // TODO Future Jimmy, please document how you did the arm in Phoenix Tuner X
    public static final int upperCoder = 4;
    public static final int lowCoder = 5;

    // Offsets are the CANcoder absolute position in the calibration position
    public static final double lowerOffset = 151.9;
    public static final double highOffset = 17.4;

    // Voltage to apply when moving arm manually
    public static final double manualVoltage = 1.5;
  }

  public static final class Intake {
    public static final int deviceId = 1;
    public static final double intakeCubeSpeed = 0.50;
    public static final double intakeConeSpeed = 0.90;
    public static final double outtakeCubeSpeed = 0.50;
    public static final double outtakeConeSpeed = 0.35;
    public static final double idleCubeSpeed = 0.18;
    public static final double idleConeSpeed = 0.18;
    public static final double stallCurrent = 15.0;
    public static final int stallMaxCube = 10;
    public static final int stallMaxCone = 20;  
  }

  public static final class Vision {
    public static final String limelightName = "limelight";
    public static final double maxXYError = 1.0;

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 10},
      {1.5, 0.01, 0.01, 10},
      {3, 0.145, 1.20, 30},
      {4.5, 0.75, 5.0, 90},
      {6, 1.0, 8.0, 180}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 5},
      {1.5, 0.02, 0.02, 5},
      {3, 0.04, 0.04, 15},
      {4.5, 0.1, 0.1, 30},
      {6, 0.3, 0.3, 60}
    };
  }
}