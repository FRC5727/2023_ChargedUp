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
  public static final int 
  LEFT_FRONT_DRIVE = 0, // DRIVETRAIN MOTORS
  LEFT_REAR_DRIVE = 6,
  RIGHT_FRONT_DRIVE = 2,
  RIGHT_REAR_DRIVE = 4,
  LEFT_FRONT_STEER = 1,
  LEFT_REAR_STEER = 7,
  RIGHT_FRONT_STEER = 3,
  RIGHT_REAR_STEER = 5;






public static final int 
LEFT_FRONT_ENCODER = 0, // CANCODERS
  LEFT_REAR_ENCODER = 3,
  RIGHT_FRONT_ENCODER = 1,
  RIGHT_REAR_ENCODER = 2;

//public static final int CANDLE = 19;

public static final int MAX_COUNTS_PER_REV = 42;
public static final double EPSILON = 0.0001;

// The left-to-right distance between the drivetrain wheels. Should be measured from center to center.
public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5; // Measure and set trackwidth
// The front-to-back distance between the drivetrain wheels. Should be measured from center to center.
public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5; // Measure and set wheelbase

public static final double LEFT_FRONT_STEER_OFFSET = Math.toRadians(-39.814453); // Measure and set front left steer offset
public static final double LEFT_REAR_STEER_OFFSET = Math.toRadians(-198.457031); // Measure and set front right steer offset
public static final double RIGHT_FRONT_STEER_OFFSET = Math.toRadians(-239.589844); // Measure and set back left steer offset
//FR is correct
public static final double RIGHT_REAR_STEER_OFFSET = Math.toRadians(-249.697266); // Measure and set back right steer offset
/*
 * public static double fleo = Math.toRadians(-39.814453);
    public static double freo = Math.toRadians(-239.589844);
    public static double rreo = Math.toRadians(-249.697266);
    public static double rleo = Math.toRadians(-198.457031);
public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(-170.068359375); // Measure and set front left steer offset
public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(-19.119522094726562); // Measure and set front right steer offset
public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(-349.541015625); // Measure and set back left steer offset
//FR is correct
public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(-260.859375); 
 */
}//intake front

