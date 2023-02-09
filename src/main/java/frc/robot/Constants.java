// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


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

  // public static final double fleo = Math.toRadians(-180.615); //Front Left Encoder Offset
  // public static final double freo = Math.toRadians(-123.223); //Front Right Encoder Offset
  // public static final double rreo = Math.toRadians(-6.500); //Rear Right Encoder Offset
  // public static final double rleo = Math.toRadians(-357.360); //Rear Left Encoder Offset
  

  public static final double fleo = Math.toRadians(-225.443); //Front Left Encoder Offset
  public static final double freo = Math.toRadians(-0.081); //Front Right Encoder Offset
  public static final double rreo = Math.toRadians(-251.190); //Rear Right Encoder Offset
  public static final double rleo = Math.toRadians(-196.433); //Rear Left Encoder Offset
  //Turn wheels to 0
  //Get encoder abs offset
  //input values
  //push code
  //turn off
  //turn on
  //verify





  // Offset in degrees
  public static final double angleOffset = 0;

  // Gear ratio
  public static final double gearRatio = 450; //450:1 on the lower arm

  // Profiled PID controller constants
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  // Arm feedforward constants
  public static final double kS = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  // Constraints for motion profiling
  public static final double velConstraint = 0.5;
  public static final double accelConstraint = 0.5; //setting this to 0.5 for testing purposes 

  // Offset in degrees
  public static final double angleOffsetH = 0;

  // Gear ratio
  public static final double gearRatioH = 450; //450:1 on the lower arm

  // Profiled PID controller constants
  public static final double kPH = 0;
  public static final double kIH = 0;
  public static final double kDH = 0;

  // Arm feedforward constants
  public static final double kSH = 0;
  public static final double kGH = 0;
  public static final double kVH = 0;
  public static final double kAH = 0;

  // Constraints for motion profiling
  public static final double velConstraintH = 0.5;
  public static final double accelConstraintH = 0.5; //setting this to 0.5 for testing purposes 


  public static final String rickBot = "CANivore";

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
  

  public static final double MAX_VOLTAGE = 13.0;

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
  public static double maxAngularVelocity = maxVelocity / Math.hypot(swerveWidth / 2.0, swerveLength / 2.0);
  public static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularVelocity);

  
  
  // private static final TalonFX FLDMTalon = new TalonFX(fldmPort, rickBot); //Front Left Drive Motor
  // private static final TalonFX FLSMTalon = new TalonFX(flsmPort, rickBot); // Front Left Steer Motor
  // private static final TalonFX FRDMTalon = new TalonFX(frdmPort, rickBot); //Front Right Drive Motor
  // private static final TalonFX FRSMTalon = new TalonFX(frsmPort, rickBot); // Front Right Steer Motor
  // private static final TalonFX RRDMTalon = new TalonFX(rrdmPort, rickBot); //Rear Right Drive Motor
  // private static final TalonFX RRSMTalon = new TalonFX(rrsmPort, rickBot); //Rear Right Steer Motor
  // private static final TalonFX RLDMTalon = new TalonFX(rldmPort, rickBot); //Rear Left Drive Motor
  // private static final TalonFX RLSMTalon = new TalonFX(rlsmPort, rickBot); //Rear Left Steer Motor

  // public static final CANCoder FLEcanCoderConstantsWithCANivore = new CANCoder(flePort, rickBot); //Front Left Encoder Port
  // public static final CANCoder FREcanCoderConstantsWithCANivore = new CANCoder(frePort, rickBot); //Front Right Encoder Port
  // public static final CANCoder RREcanCoderConstantsWithCANivore = new CANCoder(rrePort, rickBot); //Rear Right Encoder Port
  // public static final CANCoder RLEcanCoderConstantsWithCANivore = new CANCoder(rlePort, rickBot); //Rear Left Encoder Port
   
  /*
    FLDMTalon
    FLSMTalon
    FRDMTalon
    FRSMTalon
    RRDMTalon
    RRSMTalon
    RLDMTalon
    RLSMTalon
   */

}

