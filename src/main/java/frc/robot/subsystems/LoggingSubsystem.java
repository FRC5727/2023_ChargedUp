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
  }

  public void updateArmLogs(){
    // LogManager.addDataType("Lower|Higher: Arm/function", value);
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

  public void updateRobotPositionLogs(){
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
    LogManager.addDoubleArray("Odometry/Pose2d", pose);
    LogManager.addDoubleArray("Odometry/UpdatedPose2d", updatedPose);

    LogManager.addDouble("Odometry/Gyro/Yaw", s_Swerve.getYaw().getDegrees());
    LogManager.addDouble("Odometry/Gyro/Pitch", s_Swerve.getPitch());
    LogManager.addDouble("Odometry/Gyro/Roll", s_Swerve.getRoll());

    LogManager.addDouble("Odometry/Gyro/X", s_RobotPosition.getPose().getX());
    LogManager.addDouble("Odometry/Gyro/Y", s_RobotPosition.getPose().getY());

  }
  @Override
  public void periodic() {
    updateSwerveLogs();
    updateArmLogs();
    updateIntakeLogs();
    updateRobotPositionLogs();
  }
}
