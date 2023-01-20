// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  
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

  public static final int dXboxA = 1, dXboxB = 2, dXboxX = 3, dXboxY = 4; 

  public static final int dXboxController = 0, mXboxController = 1;



//public static final int CANDLE = 19;

public static final int MAX_COUNTS_PER_REV = 42;
public static final double EPSILON = 0.0001;

// The left-to-right distance between the drivetrain wheels. Should be measured from center to center.
public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // Measure and set trackwidth
// The front-to-back distance between the drivetrain wheels. Should be measured from center to center.
public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // Measure and set wheelbase


}

