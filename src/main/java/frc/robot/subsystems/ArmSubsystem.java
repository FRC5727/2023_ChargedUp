// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.library.math.Conversions;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonFX lowerArmMaster;
  private WPI_TalonFX lowerArmSlave;

  private WPI_TalonFX highArmMaster;
  private WPI_TalonFX highArmSlave;

  private CANCoder lowerArmCoder;
  private CANCoder highArmCoder;

  private double lowArmAngleOffset;
  private double highArmAngleOffset;

  private double lowerArmGearRatio = 66.0 / 16.0;
  private double highArmGearRatio = 44.0 / 16.0;

  // private ProfiledPIDController lowPidController;
  // private ProfiledPIDController highPidController;
  private PIDController lowPidController;
  private PIDController highPidController;

  private ArmFeedforward highArmFeedforward;
  private ArmFeedforward lowArmFeedforward;

  private double L_kp = 0.90,
      L_ki = 0.00,
      L_kd = 0.10;

  private double H_kp = 0.10,
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

  private double L_maxVelocity, L_maxAcceleration;
  private double H_maxVelocity, H_maxAcceleration;
  private double L_maxVoltage = 5;
  private double H_maxVoltage = 5;

  private double LowerCANcoderInitTime = 0.0;
  private double HighCANcoderInitTime = 0.0;

  public ArmSubsystem() {
    this.lowerArmMaster = new WPI_TalonFX(9, Constants.rickBot);
    this.lowerArmSlave = new WPI_TalonFX(11, Constants.rickBot);
    this.lowerArmCoder = new CANCoder(4, Constants.rickBot);

    this.highArmMaster = new WPI_TalonFX(8, Constants.rickBot);
    this.highArmSlave = new WPI_TalonFX(10, Constants.rickBot);
    this.highArmCoder = new CANCoder(5, Constants.rickBot);

    this.lowPidController = new PIDController(L_kp, L_ki, L_kd);
    // new TrapezoidProfile.Constraints(0.025, 0.025));

    this.highPidController = new PIDController(H_kp, H_ki, H_kd);
    // new TrapezoidProfile.Constraints(0.025, 0.025));

    this.lowArmFeedforward = new ArmFeedforward(L_ks, L_kg, L_kv, L_ka);
    this.highArmFeedforward = new ArmFeedforward(H_ks, H_kg, H_kv, H_ka);

    this.lowArmAngleOffset = 0;
    this.highArmAngleOffset = 287.92;

    this.lowerArmGearRatio = 0.40; // 4.125
    this.highArmGearRatio = 0.10;

    this.L_maxVelocity = 0.025; // Degrees per second
    this.L_maxAcceleration = 0.025; // Degrees per second squared

    this.H_maxVelocity = 0.025; // Degrees per second
    this.H_maxAcceleration = 0.025; // Degrees per second squared

    // configArmMotor();
    // setCurrentPosToGoal();

    lowPidController.disableContinuousInput();
    lowPidController.setTolerance(5); // , 5);

    highPidController.disableContinuousInput();
    highPidController.setTolerance(5); // , 5);

    lowerArmSlave.follow(lowerArmMaster);
    highArmSlave.follow(highArmMaster);
    lowerArmSlave.setInverted(true);
    highArmSlave.setInverted(true);
    resetToAbsolute();
  }

  private double constrainValue(double value, double max) {
    return Math.signum(value) * Math.min(Math.abs(value), max);
  }
  // public void configArmMotor(){
  // lowerArmSlave.follow(lowerArmMaster);
  // highArmSlave.follow(highArmMaster);

  // lowerArmMaster.configFactoryDefault();
  // lowerArmSlave.configFactoryDefault();

  // highArmMaster.configFactoryDefault();
  // highArmSlave.configFactoryDefault();

  // lowerArmSlave.setInverted(true);
  // highArmSlave.setInverted(true);
  // }
  // private void waitForLowCanCoder(){
  // /*
  // * Wait for up to 1000 ms for a good CANcoder signal.
  // *
  // * This prevents a race condition during program startup
  // * where we try to synchronize the Falcon encoder to the
  // * CANcoder before we have received any position signal
  // * from the CANcoder.
  // */
  // for (int i = 0; i < 100; ++i) {
  // lowerArmCoder.getAbsolutePosition();
  // if (lowerArmCoder.getLastError() == ErrorCode.OK) {
  // break;
  // }
  // Timer.delay(0.010);
  // LowerCANcoderInitTime += 10;
  // }
  // }
  // private void waitForHighCanCoder(){
  // /*
  // * Wait for up to 1000 ms for a good CANcoder signal.
  // *
  // * This prevents a race condition during program startup
  // * where we try to synchronize the Falcon encoder to the
  // * CANcoder before we have received any position signal
  // * from the CANcoder.
  // */
  // for (int i = 0; i < 100; ++i) {
  // highArmCoder.getAbsolutePosition();
  // if (highArmCoder.getLastError() == ErrorCode.OK) {
  // break;
  // }
  // Timer.delay(0.010);
  // HighCANcoderInitTime += 10;
  // }
  // }
  // public Rotation2d getHighCanCoder(){
  // return Rotation2d.fromDegrees(highArmCoder.getAbsolutePosition());
  // }
  // public Rotation2d getLowCanCoder(){
  // return Rotation2d.fromDegrees(highArmCoder.getAbsolutePosition());
  // }

  // public double getHighAbsoluteAngle(){
  // return Conversions.CANcoderToDegrees(highArmCoder.getAbsolutePosition(),
  // highArmGearRatio);
  // }
  // public double getLowAbsoluteAngle(){
  // return Conversions.CANcoderToDegrees(lowerArmCoder.getAbsolutePosition(),
  // lowerArmGearRatio);
  // }

  public double getLowRelativeAngle() {
    return lowerArmCoder.getPosition() * (360.0 / (lowerArmGearRatio * 4096.0));
  }

  public double getHighRelativeAngle() {
    return highArmCoder.getPosition() * (360.0 / (highArmGearRatio * 4096.0));
  }

  // public double getHighAngle(){
  // return getHighAbsoluteAngle() - highArmAngleOffset;
  // }
  // public double getLowAngle(){
  // return getLowAbsoluteAngle() - lowArmAngleOffset;
  // }

  // public void setHighArmGoal(double angle){
  // highPidController.setGoal(angle);
  // }
  // public void setLowArmGoal(double angle){
  // lowPidController.setGoal(angle);
  // }

  public void chassisPos() {
    lowPidController.setSetpoint(0);
    highPidController.setSetpoint(0);
  }

  public void intakeGroundPos() {
    lowPidController.setSetpoint(36);
    highPidController.setSetpoint(-29);
  }

  public void highPos() {
    lowPidController.setSetpoint(54);
    highPidController.setSetpoint(86);
  }

  public void midPos() {
    lowPidController.setSetpoint(26);
    highPidController.setSetpoint(121);
  }

  public void lowPos() {
    lowPidController.setSetpoint(27);
    highPidController.setSetpoint(2);
  }

  // public double getHighGoal(){
  // return highPidController.getGoal().position;
  // }
  // public double getLowGoal(){
  // return lowPidController.getGoal().position;
  // }
  // public void setCurrentPosToGoal() {
  // lowPidController.setGoal(getLowRelativeAngle());
  // highPidController.setGoal(getHighRelativeAngle());
  // }
  public void resetToAbsolute() {
    // waitForCanCoder();

    double absolutePosition1 = Conversions.degreesToFalcon(getLowRelativeAngle(), lowerArmGearRatio);
    lowerArmMaster.setSelectedSensorPosition(absolutePosition1);
    lowerArmSlave.setSelectedSensorPosition(absolutePosition1);

    double absolutePosition2 = Conversions.degreesToFalcon(getHighRelativeAngle(), highArmGearRatio);
    highArmMaster.setSelectedSensorPosition(absolutePosition2);
    highArmSlave.setSelectedSensorPosition(absolutePosition2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // highArmMaster.setVoltage( //sets the voltage according to the PID controller
    // and armFeedForward
    // highPidController.calculate(getHighRelativeAngle(), 86)); //getHighGoal()
    // //86

    lowerArmMaster.setVoltage(
        constrainValue(
            lowPidController.calculate(getLowRelativeAngle(), 26), L_maxVoltage));

    SmartDashboard.putNumber("Low Relative Angle (CANcoder): ", getLowRelativeAngle());
    SmartDashboard.putNumber("High Relative Angle (CANcoder): ", getHighRelativeAngle());

    // SmartDashboard.putNumber("Low Arm Goal",
    // lowPidController.getGoal().position);
    // SmartDashboard.putNumber("High Arm Goal",
    // highPidController.getGoal().position);

    SmartDashboard.putNumber("Low Arm Master Voltage:", lowerArmMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("Low Arm Slave Voltage:", lowerArmSlave.getMotorOutputVoltage());

    SmartDashboard.putNumber("High Arm Master Voltage:", highArmMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("High Arm Slave Voltage:", highArmSlave.getMotorOutputVoltage());

  }
}
// + //PID calculates according to its current angle, and the goal angle its
// trying to get to
// highArmFeedforward.calculate(
// highPidController.getSetpoint().position,
// highPidController.getSetpoint().velocity)
// // +
// // lowArmFeedforward.calculate(
// // lowPidController.getSetpoint().position,
// // lowPidController.getSetpoint().velocity)
