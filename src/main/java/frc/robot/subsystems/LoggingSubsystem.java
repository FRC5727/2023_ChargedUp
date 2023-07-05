// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.util.logging.LogManager;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.RobotPosition;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggingSubsystem extends SubsystemBase {
  private LED s_Led;
  private IntakeSubsystem s_Intake;
  private Swerve s_Swerve;
  private RobotPosition s_RobotPosition;
  private ArmSubsystem s_Arm;
  /** Creates a new LoggingSubsystem. */
  public LoggingSubsystem(LED s_Led, IntakeSubsystem s_Intake, Swerve s_Swerve, RobotPosition s_RobotPosition, ArmSubsystem s_Arm) {
    this.s_Led = s_Led;
    this.s_Intake = s_Intake;
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    this.s_RobotPosition = s_RobotPosition;
  }
  public void updateSwerveLogs() {
    double[] actualStates = {
      s_Swerve.mSwerveMods[0].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[0].getState().speedMetersPerSecond,
      s_Swerve.mSwerveMods[1].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[1].getState().speedMetersPerSecond,
      s_Swerve.mSwerveMods[2].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[2].getState().speedMetersPerSecond,
      s_Swerve.mSwerveMods[3].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[3].getState().speedMetersPerSecond
    };
    LogManager.addDoubleArray("Swerve/actual swerve states", actualStates);
    double[] desiredStates = {
      s_Swerve.mSwerveMods[0].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[0].getDesiredVelocity(),
      s_Swerve.mSwerveMods[1].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[1].getDesiredVelocity(),
      s_Swerve.mSwerveMods[2].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[2].getDesiredVelocity(),
      s_Swerve.mSwerveMods[3].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[3].getDesiredVelocity()
    };
    LogManager.addDoubleArray("Swerve/desired swerve states", desiredStates);
    double[] pose = {
      s_RobotPosition.getPose().getX(),
      s_RobotPosition.getPose().getY(),
      s_RobotPosition.getPose().getRotation().getDegrees()
    };
    double[] updatedPose = {
      s_RobotPosition.getUpdatedPose().getX(),
      s_RobotPosition.getUpdatedPose().getY(),
      s_RobotPosition.getUpdatedPose().getRotation().getDegrees()
    };
    LogManager.addDoubleArray("Swerve/Pose2d", pose);
    LogManager.addDoubleArray("Swerve/UpdatedPose2d", updatedPose);

    LogManager.addDouble("Swerve/Gyro/Yaw", s_Swerve.getYaw().getDegrees());
    LogManager.addDouble("Swerve/Gyro/Pitch", s_Swerve.getPitch());
    LogManager.addDouble("Swerve/Gyro/Roll", s_Swerve.getRoll());

    LogManager.addDouble("Swerve/Pose/X", s_RobotPosition.getPose().getX());
    LogManager.addDouble("Swerve/Pose/Y", s_RobotPosition.getPose().getY());
  }

  public void updateArmLogs(){
    LogManager.addDouble("Arm/Lower/Goal", s_Arm.getLowerGoal());
    LogManager.addDouble("Arm/Upper/Goal", s_Arm.getUpperGoal());

    LogManager.addBoolean("Arm/Lower/isSet", s_Arm.isLowerSet());
    LogManager.addBoolean("Arm/Upper/isSet", s_Arm.isUpperSet());

    LogManager.addDouble("Arm/Lower/AngleAbsolute", s_Arm.getLowerAbsoluteAngle());
    LogManager.addDouble("Arm/Upper/AngleAbsolute", s_Arm.getUpperAbsoluteAngle());

    LogManager.addDouble("Arm/Lower/Angle", s_Arm.getLowerAngle());
    LogManager.addDouble("Arm/Upper/Angle", s_Arm.getUpperAngle());

    LogManager.addDouble("Arm/Lower/Master/Voltage", s_Arm.getLowerMasterVoltage());
    LogManager.addDouble("Arm/Upper/Master/Voltage", s_Arm.getUpperMasterVoltage());

    LogManager.addDouble("Arm/Lower/Slave/Voltage", s_Arm.getLowerSlaveVoltage());
    LogManager.addDouble("Arm/Upper/Slave/Voltage", s_Arm.getUpperSlaveVoltage());

    LogManager.addDouble("Arm/Lower/PID/P-value", s_Arm.getLowerPValue());
    LogManager.addDouble("Arm/Upper/PID/P-value", s_Arm.getUpperPValue());
  }
  
  public void updateIntakeLogs(){
    LogManager.addBoolean("Intake/States/Cube?", s_Intake.isCube());
    LogManager.addBoolean("Intake/States/isIdle", s_Intake.isIdle());
    LogManager.addBoolean("Intake/States/Piece captured?", s_Intake.isStalled());
    LogManager.addBoolean("Intake/States/isIntaking", s_Intake.isIntaking());

    LogManager.addDouble("Intake/Current/Output", s_Intake.getIntakeOutputCurrent());
    LogManager.addDouble("Intake/Current/Speed", s_Intake.getIntakeCurrentSpeed());

    LogManager.addDouble("Intake/States/StallCounter", s_Intake.getStallCounter());
  }

  
  public void updateHardwareStatLogs(){
    /*================================================================================= */
    //ARM Monitoring
    //Lower Master
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Lower/Master/Stats/Temp", 
    s_Arm.lowerArmMasterFx().getTemperature());
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Lower/Master/Stats/BusVoltage", 
    s_Arm.lowerArmMasterFx().getBusVoltage());
    //Lower Slave
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Lower/Slave/Stats/Temp", 
    s_Arm.lowerArmSlaveFx().getTemperature());
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Lower/Slave/Stats/BusVoltage", 
    s_Arm.lowerArmSlaveFx().getBusVoltage());
    //Upper Master
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Upper/Master/Stats/Temp", 
    s_Arm.upperArmMasterFx().getTemperature());
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Upper/Master/Stats/BusVoltage", 
    s_Arm.upperArmMasterFx().getBusVoltage());
    //Upper Slave
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Upper/Slave/Stats/Temp", 
    s_Arm.upperArmSlaveFx().getTemperature());
    LogManager.addDouble("Hardware/Arm/Motor/Falcon500/Upper/Slave/Stats/BusVoltage", 
    s_Arm.upperArmSlaveFx().getBusVoltage());
    //Lower CANCoder
    LogManager.addDouble("Hardware/Arm/CANCoder/Lower/Stats/BusVoltage", 
    s_Arm.lowerCoderFx().getBusVoltage());
    //Upper CANCoder
    LogManager.addDouble("Hardware/Arm/CANCoder/Upper/Stats/BusVoltage", 
    s_Arm.upperCoderFx().getBusVoltage());
    /*================================================================================= */
    //Intake Neo
    LogManager.addDouble("Hardware/Intake/Motor/Neo550/Stats/Temp", 
    s_Intake.intakeNeoFx().getMotorTemperature());
    LogManager.addDouble("Hardware/Intake/Motor/Neo550/Stats/BusVoltage", 
    s_Intake.intakeNeoFx().getBusVoltage());
    /*================================================================================= */
    //Swerve Motors
    // 0 1
    // 2 3
    //DRIVE MOTORS
    // Front Left Drive Motor (module 0)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Drive/FL/Stats/Temp", 
    s_Swerve.mSwerveMods[0].getDriveMotorTemp());
    // Front Right Drive Motor (module 1)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Drive/FR/Stats/Temp", 
    s_Swerve.mSwerveMods[1].getDriveMotorTemp());
    // Rear Left Drive Motor (module 2)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Drive/RL/Stats/Temp", 
    s_Swerve.mSwerveMods[2].getDriveMotorTemp());
    // Rear Right Drive Motor (module 3)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Drive/RR/Stats/Temp", 
    s_Swerve.mSwerveMods[3].getDriveMotorTemp());
    /*================================================================================= */
    //ANGLE MOTORS
    // Front Left Angle Motor (module 0)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Angle/FL/Stats/Temp", 
    s_Swerve.mSwerveMods[0].getAngleMotorTemp());
    // Front Right Angle Motor (module 1)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Angle/FR/Stats/Temp", 
    s_Swerve.mSwerveMods[1].getAngleMotorTemp());
    // Rear Left Angle Motor (module 2)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Angle/RL/Stats/Temp", 
    s_Swerve.mSwerveMods[2].getAngleMotorTemp());
    // Rear Right Angle Motor (module 3)
    LogManager.addDouble("Hardware/Swerve/Falcon500/Angle/RR/Stats/Temp", 
    s_Swerve.mSwerveMods[3].getAngleMotorTemp());
    /*================================================================================= */
    //Pigeon 2
    LogManager.addDouble("Hardware/Swerve/Pigeon2/Stats/Temp", 
    s_Swerve.gyroFx().getTemp());
    /*================================================================================= */
    //CANdle
    LogManager.addDouble("Hardware/LEDs/CANdle/Stats/Temp", 
    s_Led.caNdlefx().getTemperature());
    LogManager.addDouble("Hardware/LEDs/CANdle/Stats/BusVoltage", 
    s_Led.caNdlefx().getBusVoltage());
    LogManager.addDouble("Hardware/LEDs/CANdle/Stats/Current", 
    s_Led.caNdlefx().getCurrent());
    LogManager.addDouble("Hardware/LEDs/CANdle/Stats/5vRailVoltage", 
    s_Led.caNdlefx().get5VRailVoltage());
    /*================================================================================= */

  }

  @Override
  public void periodic() {
    updateSwerveLogs();
    updateArmLogs();
    updateIntakeLogs();
    updateHardwareStatLogs();
  }
}