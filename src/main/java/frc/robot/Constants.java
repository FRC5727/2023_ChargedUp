// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static int fldmPort = 0; //Front Left Drive Motor
  public static int flsmPort = 1; // Front Left Steer Motor

  public static int frdmPort = 2; //Front Right Drive Motor
  public static int frsmPort = 3; // Front Right Steer Motor

  public static int rrdmPort = 4; //Rear Right Drive Motor
  public static int rrsmPort = 5; //Rear Right Steer Motor

  public static int rldmPort = 6; //Rear Left Drive Motor
  public static int rlsmPort = 7; //Rear Left Steer Motor

  public static int flePort = 0; //Front Left Encoder Port
  public static int frePort = 1; //Front Right Encoder Port
  public static int rrePort = 2; //Rear Right Encoder Port
  public static int rlePort = 3; //Rear Left Encoder Port

  public static double fleo = Math.toRadians(-39.814453); //Front Left Encoder Offset
  public static double freo = Math.toRadians(-239.589844); //Front Right Encoder Offset
  public static double rreo = Math.toRadians(-249.697266); //Rear Right Encoder Offset
  public static double rleo = Math.toRadians(-198.457031); //Rear Left Encoder Offset
//intake front

  public static PIDController translationXController = new PIDController(0, 0, 0); //10
  public static PIDController translationYController = new PIDController(0, 0, 0);
  public static PIDController rotationController = new PIDController(0, 0, 0);

  public static double maxVelocity = (6380.0 / 60.0 * 
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() * 
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * 
        Math.PI);
  

  public static XboxController dXboxController = new XboxController(0);
  public static XboxController mXboxController = new XboxController(1);

  public static double deadzone = 0.05;

  public static int talonCount = 8;
  
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 13.0;

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
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  public static final double speedLimit = 0.75;
  public static final double rSpeedLimit = 0.40;
  //Buttons
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
  public static int XboxLeftXstick = 0;
  public static int XboxRightXstick = 4;
  public static int XboxLeftYstick = 1;
  public static int XboxRightYstick = 5;

  //Triggers
  public static int XboxLeftTriger = 2;
  public static int XboxRightTriger = 3;


  public static double translationRateLimit = 2.5;
  public static double rotationRateLimit = 2.5;

  public static double controllerXYExpo = 2.6;
  public static double controllerRoExpo = 2.6;
        //x^3
        //x^1.96 the best so far
        //x^3.4
        //x^2.6
        //x^4.6
        //x^5.4
        //x^0.6


//public static final int CANDLE = 19;

  public static final int MAX_COUNTS_PER_REV = 42;
  public static final double EPSILON = 0.0001;

  // The left-to-right distance between the drivetrain wheels. Should be measured from center to center.
  public static final double swerveWidth = 0.635; // Measure and set trackwidth
  // The front-to-back distance between the drivetrain wheels. Should be measured from center to center.
  public static final double swerveLength = 0.7366; // Measure and set wheelbase

  /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static double maxAngularVelocity = maxVelocity / Math.hypot(swerveWidth / 2.0, swerveLength / 2.0);

  public static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularVelocity);

  



}

