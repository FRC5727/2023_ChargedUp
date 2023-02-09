// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.library.math.Conversions;
import frc.robot.Constants;

public class lowerArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonFX m_armMotor;
  private CANCoder m_encoder;

  // Encoder angle offset
  private double m_angleOffset;

  // Gear ratio of arm
  private double m_gearRatio;

  private ProfiledPIDController m_pidController;
  private ArmFeedforward m_feedForwardController;
  
  private double kp, ki, kd;

  private double ks, kg, kv, ka;

  private double maxVelocity, maxAcceleration;
  
  public lowerArmSubsystem() {
    this.m_armMotor = new WPI_TalonFX(0, "");
    this.m_encoder = new CANCoder(0, "");

    this.m_pidController = new ProfiledPIDController(kp, ki, kd, 
    new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    this.m_feedForwardController = new ArmFeedforward(ks, kg, kv, ka);

    this.m_angleOffset = Constants.angleOffset;

    this.m_gearRatio = Constants.gearRatio;
    
    this.maxVelocity = Constants.velConstraint;
    this.maxAcceleration = Constants.accelConstraint;

  }
  public double getAbsoluteAngle() {
    return Conversions.CANcoderToDegrees(m_encoder.getAbsolutePosition(), m_gearRatio);
  }

  public double getAngle() {
    return getAbsoluteAngle()-m_angleOffset;
  }
  
  public void setGoal(double angle) {
    m_pidController.setGoal(angle); //set to angle we want to use for which position 
  }
  
  public double getGoal() {
    return m_pidController.getGoal().position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_armMotor.setVoltage(
      /** PID Controller calculates output based on 
      the current position and the goal **/
      m_pidController.calculate(getAngle()) 
      /** Feedforward uses setpoints calculated by 
      motion profiling **/
    + m_feedForwardController.calculate(
      m_pidController.getSetpoint().position, 
    m_pidController.getSetpoint().velocity));
  }
}
