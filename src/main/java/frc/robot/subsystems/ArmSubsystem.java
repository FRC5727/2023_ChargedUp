// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.library.math.Conversions;
import frc.robot.Constants;

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

  private double lowerArmGearRatio;
  private double highArmGearRatio;

  private ProfiledPIDController lowPidController;
  private ProfiledPIDController highPidController;

  private ArmFeedforward highArmFeedforward;
  private ArmFeedforward lowArmFeedforward;

  private double  L_kp,
                  L_ki,
                  L_kd;

  private double  H_kp,
                  H_ki,
                  H_kd;

  private double  L_ks, 
                  L_kg, //0.16
                  L_kv, //8.08
                  L_ka; //0.03
          
  private double  H_ks, 
                  H_kg, //0.21
                  H_kv, //4.45
                  H_ka; //0.02

  private double L_maxVelocity, L_maxAcceleration;
  private double H_maxVelocity, H_maxAcceleration;

  public ArmSubsystem() {
    this.lowerArmMaster = new WPI_TalonFX(Constants.lowerMaster, Constants.rickBot);
    this.lowerArmSlave = new WPI_TalonFX(Constants.lowerSlave, Constants.rickBot);
    this.lowerArmCoder = new CANCoder(0, Constants.rickBot);

    this.highArmMaster = new WPI_TalonFX(Constants.highMaster, Constants.rickBot);
    this.highArmSlave = new WPI_TalonFX(Constants.highSlave, Constants.rickBot);
    this.highArmCoder = new CANCoder(0, Constants.rickBot);

    this.lowPidController = new ProfiledPIDController(L_kp, L_ki, L_kd, 
    new TrapezoidProfile.Constraints(L_maxVelocity, L_maxAcceleration));

    this.highPidController = new ProfiledPIDController(H_kp, H_ki, H_kd, 
    new TrapezoidProfile.Constraints(H_maxVelocity, H_maxAcceleration));

    this.lowArmFeedforward = new ArmFeedforward(L_ks, L_kg, L_kv, L_ka);
    this.highArmFeedforward = new ArmFeedforward(H_ks, H_kg, H_kv, H_ka);

    this.lowArmAngleOffset = Constants.angleOffsetL;
    this.highArmAngleOffset = Constants.angleOffsetH; 

    this.lowerArmGearRatio = Constants.gearRatioL; 
    this.highArmGearRatio = Constants.gearRatioH;

    this.L_maxVelocity = 0.5;
    this.L_maxAcceleration = 0.5;
    
    this.H_maxVelocity = 0.5;
    this.H_maxAcceleration = 0.5;

    configArmMotor();
  }
  public void configArmMotor(){
    lowerArmSlave.follow(lowerArmMaster);
    highArmSlave.follow(highArmMaster);

    lowerArmSlave.setInverted(true);
    highArmSlave.setInverted(true);
  }

  public double getHighAbsoluteAngle(){
    return Conversions.CANcoderToDegrees(highArmCoder.getAbsolutePosition(), highArmGearRatio); 
  }
  public double getLowAbsoluteAngle(){
    return Conversions.CANcoderToDegrees(lowerArmCoder.getAbsolutePosition(), lowerArmGearRatio); 
  }

  public double getHighAngle(){
    return getHighAbsoluteAngle()- highArmAngleOffset;
  }
  public double getLowAngle(){
    return getLowAbsoluteAngle()- lowArmAngleOffset;
  }

  public void setHighArmGoal(double angle){
    highPidController.setGoal(angle);
  }
  public void setLowArmGoal(double angle){
    lowPidController.setGoal(angle);
  }

  public double getHighGoal(){
    return highPidController.getGoal().position;
  }
  public double getLowGoal(){
    return lowPidController.getGoal().position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      highArmMaster.setVoltage(
      highPidController.calculate(getHighAngle(), getHighGoal()) + 

      highArmFeedforward.calculate(
      highPidController.getSetpoint().position, 
      highPidController.getSetpoint().velocity));
    
      lowerArmMaster.setVoltage(
      lowPidController.calculate(getLowAngle(), getLowGoal()) + 

      lowArmFeedforward.calculate(
      lowPidController.getSetpoint().position, 
      lowPidController.getSetpoint().velocity)
    );
    SmartDashboard.putNumber("High Arm Angle: ", getHighAngle());
    SmartDashboard.putNumber("Low Arm Angle: ", getLowAngle());
  }
}
