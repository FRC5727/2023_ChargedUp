// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.omegabytes.library.omegaSwerveLib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static int pigeon2IMU = 0;

  public static final int fldmPort = 0; //Front Left Drive Motor
  public static final int flsmPort = 1; // Front Left Steer Motor

  public static final int frdmPort = 2; //Front Right Drive Motor
  public static final int frsmPort = 3; // Front Right Steer Motor

  public static final int rrdmPort = 4; //Rear Right Drive Motor
  public static final int rrsmPort = 5; //Rear Right Steer Motor

  public static final int rldmPort = 6; //Rear Left Drive Motor
  public static final int rlsmPort = 7; //Rear Left Steer Motor

  public static final int flePort = 0; //Front Left Encoder Port
  public static final int frePort = 1; //Front Right Encoder Port
  public static final int rrePort = 2; //Rear Right Encoder Port
  public static final int rlePort = 3; //Rear Left Encoder Port

  public static final int highMaster = 8;
  public static final int highSlave = 10;

  public static final int lowerMaster = 9;
  public static final int lowerSlave = 11;

  public static final int lowerArmCoder = 4;
  public static final int highArmCoder = 5;

  
  public static final double fleo = -Math.toRadians(316.66); //Front Left Encoder Offset
  public static final double freo = -Math.toRadians(247.14); //Front Right Encoder Offset
  public static final double rreo = -Math.toRadians(90.70); //Rear Right Encoder Offset
  public static final double rleo = -Math.toRadians(208.82); //Rear Left Encoder Offset
  //Turn wheels to 0
  //Set all Offsets to 0
  //Push Code
  //Power Cycle
  //Get encoder abs value
  //input values
  //push code
  //Power Cycle
  //verify


  public static final String rickBot = "CANivore";

  public static PIDController translationXController = new PIDController(0.01, 0, 0); //10
  public static PIDController translationYController = new PIDController(0.01, 0, 0);
  public static PIDController rotationController = new PIDController(0.01, 0, 0);

  public static double maxVelocity = (6380.0 / 60.0 * 
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() * 
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * 
        Math.PI);
  

  public static XboxController dXboxController = new XboxController(0);
  public static XboxController mXboxController = new XboxController(1);

  public static double deadzone = 0.05;

  public static int talonCount = 8;
  

  public static final double MAX_VOLTAGE = 13.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  //Buttons
  // TODO Why do you we need these?  They are defined in the XBoxController class
  public static final int aXboxButton = 1;
  public static final int bXboxButton = 2;
  public static final int xXboxButton = 3;
  public static final int yXboxButton = 4;
  public static final int lbXboxBumper = 5;
  public static final int rbXboxBumper = 6;
  public static final int backXboxButton = 7;
  public static final int startXboxButton = 8;
  public static final int leftStickXboxButton = 9;
  public static final int rightStickXboxButton = 10;

  //Axis / sticks
  public static final int XboxLeftXstick = 0;
  public static final int XboxLeftYstick = 1;
  public static final int XboxLeftTriger = 2;
  public static final int XboxRightTriger = 3;
  public static final int XboxRightXstick = 4;
  public static final int XboxRightYstick = 5;

  //POV
  public static final int povUp = 0;
  public static final int povUpRight = 45;
  public static final int povRight = 90;
  public static final int povDownRight = 135;
  public static final int povDown = 180;
  public static final int povDownLeft = 225;
  public static final int povLeft = 270;
  public static final int povUpLeft = 315;
  


  public static final double translationRateLimit = 2.5;
  public static final double rotationRateLimit = 2.5;

  // Percent of max possible speed when travelling normally
  public static final double maxTranslationPercent = 0.80;
  public static final double maxRotationPercent = 0.35;

  // Percent of max possible speed when intentionally slowing down
  public static final double slowTranslationPercent = 0.30;
  public static final double slowRotationPercent = 0.10;

  public static double controllerXYExpo = 2.6;
  public static double controllerRoExpo = 2.6;
  // TODO Revisit this logic, but use Math.abs() before exponentiation and Math.signum() to preserve sign
        //x^3
        //x^1.96 the best so far
        //x^3.4
        //x^2.6
        //x^4.6
        //x^5.4
        //x^0.6




  // The left-to-right distance between the drivetrain wheels. Should be measured from center to center.
  public static final double swerveWidth = 0.635; // Measure and set trackwidth
  // The front-to-back distance between the drivetrain wheels. Should be measured from center to center.
  public static final double swerveLength = 0.7366; // Measure and set wheelbase
  public static double maxAngularVelocity = maxVelocity / Math.hypot(swerveWidth / 2.0, swerveLength / 2.0);
  public static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularVelocity);

}

