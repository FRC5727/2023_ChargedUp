// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;
import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.cscore.CameraServerJNI.LoggerFunction;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.library.math.Conversions;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  // Abstraction of the encode positions for a defined arm position
  private class ArmPosition {
    double lowerArmAngle;
    double upperArmAngle;

    public ArmPosition(double lower, double upper) {
      lowerArmAngle = lower;
      upperArmAngle = upper;
    }
  }

  // Enumeration of all defined arm positions
  public enum Position {
    CHASSIS,
    MIDDLE,
    GRID_LOW,
    GRID_MID,
    GRID_HIGH,
    INTAKE_GROUND,
    INTAKE_SUBSTATION,
  };

  // Next position for the Arm to target
  // TODO Allow for this to be a list of positions
  private Position targetPosition = Position.CHASSIS;

  // Whether or not Arm is currently enabled to move to a position
  private boolean enabled = false;

  // Lookup "table" for each defined arm position
  private final EnumMap<Position, ArmPosition> armPositions = new EnumMap<>(Map.of(
      Position.CHASSIS, new ArmPosition(-21, -60),
      Position.MIDDLE, new ArmPosition(-25, -31),
      Position.GRID_LOW, new ArmPosition(11, -70), //og: 27, 2
      Position.GRID_MID, new ArmPosition(3, -1),
      Position.GRID_HIGH, new ArmPosition(36, 0),
      Position.INTAKE_GROUND, new ArmPosition(69, -16),
      Position.INTAKE_SUBSTATION, new ArmPosition(1, 19) //upper old: 154
    ));

  private WPI_TalonFX lowerArmMaster;
  private WPI_TalonFX lowerArmSlave;

  private WPI_TalonFX highArmMaster;
  private WPI_TalonFX highArmSlave;

  private CANCoder lowerArmCoder;
  private CANCoder highArmCoder;


  private double lowerArmGearRatio = 66.0 / 16.0;
  private double highArmGearRatio = 44.0 / 16.0;

  private PIDController lowPidController;
  private PIDController highPidController;

  private ArmFeedforward highArmFeedforward;
  private ArmFeedforward lowArmFeedforward;

  // TODO Tune this
  private double L_kp = 1.10,
      L_ki = 0.00,
      L_kd = 0.00;

  // TODO Tune this
  private double H_kp = 1.10,
      H_ki = 0.00,
      H_kd = 0.00;

  private double L_ks = 0.00,
      L_kg = 0.16, // 0.16
      L_kv = 8.08, // 8.08
      L_ka = 0.03; // 0.03

  private double H_ks = 0.00,
      H_kg = 0.21, // 0.21
      H_kv = 4.45, // 4.45
      H_ka = 0.02; // 0.02

  private double L_maxVoltage = 3;
  private double H_maxVoltage = 3;

  private double LowerCANcoderInitTime = 0.0;
  private double HighCANcoderInitTime = 0.0;

  public ArmSubsystem() {
    // TODO Get the device numbers from centrally defined constants
    this.lowerArmMaster = new WPI_TalonFX(9, Constants.CANivoreName);
    this.lowerArmSlave = new WPI_TalonFX(11, Constants.CANivoreName);
    this.lowerArmCoder = new CANCoder(Constants.lowCoder, Constants.CANivoreName);

    // TODO Get the device numbers from centrally defined constants
    this.highArmMaster = new WPI_TalonFX(8, Constants.CANivoreName);
    this.highArmSlave = new WPI_TalonFX(10, Constants.CANivoreName);
    this.highArmCoder = new CANCoder(Constants.highCoder, Constants.CANivoreName);

    this.lowPidController = new PIDController(L_kp, L_ki, L_kd);
    this.highPidController = new PIDController(H_kp, H_ki, H_kd);

    // Jimmy, why are we overriding this?  This is confusing, and I don't know why it's necessary
    this.lowerArmGearRatio = 0.40; // 4.125
    this.highArmGearRatio = 0.10;
    
    // TODO Define tolerance elsewhere
    lowPidController.disableContinuousInput();
    lowPidController.setTolerance(1, .5);

    highPidController.disableContinuousInput();
    highPidController.setTolerance(1, .5);

    lowerArmSlave.follow(lowerArmMaster);
    lowerArmSlave.setInverted(true);
    highArmSlave.follow(highArmMaster);
    highArmSlave.setInverted(false);
    highArmMaster.setInverted(true);

    lowerArmMaster.setNeutralMode(NeutralMode.Brake);
    highArmMaster.setNeutralMode(NeutralMode.Brake);
  }

  // Limit a value (positive or negative)
  private double constrainValue(double value, double max) {
    return Math.signum(value) * Math.min(Math.abs(value), max);
  }

  public double getLowRelativeAngle() {
    return lowerArmCoder.getPosition() * (360.0 / (lowerArmGearRatio * 4096.0));
  }

  public double getHighRelativeAngle() {
    return highArmCoder.getPosition() * (360.0 / (highArmGearRatio * 4096.0));
  }

  public double getLowAbsoluteAngle(){
    return lowerArmCoder.getAbsolutePosition();
  }
  public double getHighAbsoluteAngle(){
    return highArmCoder.getAbsolutePosition();
  }

  public void setTargetPosition(Position position) {
    this.targetPosition = position;
  }

  public void beginMovement() {
    lowPidController.setSetpoint(armPositions.get(targetPosition).lowerArmAngle);
    lowPidController.reset();
    highPidController.setSetpoint(armPositions.get(targetPosition).upperArmAngle);
    highPidController.reset();
  }

  public void updateMovement() {
    lowerArmMaster.setVoltage(constrainValue(lowPidController.calculate(getLowAbsoluteAngle()), L_maxVoltage));
    highArmMaster.setVoltage(constrainValue(highPidController.calculate(getHighAbsoluteAngle()), H_maxVoltage));
  }

  public void stopMovement() {
    lowerArmMaster.stopMotor();
    highArmMaster.stopMotor();
  }

  public boolean doneMovement() {
    return lowPidController.atSetpoint() && highPidController.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update dashboard to help monitor and debug

    SmartDashboard.putString("Target Position", targetPosition.toString());
    SmartDashboard.putNumber("Low Arm Setpoint: ", lowPidController.getSetpoint());
    SmartDashboard.putNumber("High Arm Setpoint: ", highPidController.getSetpoint());
    SmartDashboard.putBoolean("Low Arm set? ", lowPidController.atSetpoint());
    SmartDashboard.putBoolean("High Arm set? ", highPidController.atSetpoint());

    SmartDashboard.putNumber("Low Angle ABSOLUTE", lowerArmCoder.getAbsolutePosition());
    SmartDashboard.putNumber("High Angle ABSOLUTE", highArmCoder.getAbsolutePosition());

    SmartDashboard.putNumber("Low Arm Master Voltage:", lowerArmMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("Low Arm Slave Voltage:", lowerArmSlave.getMotorOutputVoltage());

    SmartDashboard.putNumber("High Arm Master Voltage:", highArmMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("High Arm Slave Voltage:", highArmSlave.getMotorOutputVoltage());
  }
}
