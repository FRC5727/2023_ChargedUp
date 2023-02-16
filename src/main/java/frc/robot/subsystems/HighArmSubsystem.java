// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.library.math.Conversions;
import frc.robot.Constants;

public class HighArmSubsystem extends SubsystemBase {
  /** Creates a new HighArmSubsystem. */
  private WPI_TalonFX highArmMaster;
  private WPI_TalonFX highArmSlave;

  private CANCoder highArmCoder;
  
  private double highArmAngleOffset;

  private double highArmGearRatio;

  private ProfiledPIDController highPidController;
  private ArmFeedforward highArmFeedforward;
  private double  H_kp = 0.10,
                  H_ki = 0.00,
                  H_kd = 0.00;

  private double H_maxVelocity, H_maxAcceleration;

  private double HighCANcoderInitTime = 0.0;

  
  public HighArmSubsystem() {
    this.highArmMaster = new WPI_TalonFX(Constants.highMaster, Constants.rickBot);
    this.highArmSlave = new WPI_TalonFX(Constants.highSlave, Constants.rickBot);
    this.highArmCoder = new CANCoder(5, Constants.rickBot);

    this.highPidController = new ProfiledPIDController(H_kp, H_ki, H_kd, 
    new TrapezoidProfile.Constraints(0.025, 0.025));

    this.highArmAngleOffset = 287.92; 

    this.highArmGearRatio = 0.10; 

    highPidController.disableContinuousInput();
    highPidController.setTolerance(5, 5);

  }
  public void configArmMotor(){
    // lowerArmSlave.follow(lowerArmMaster);
    highArmSlave.follow(highArmMaster);

    

    highArmMaster.configFactoryDefault();
    highArmSlave.configFactoryDefault();

    // lowerArmMaster.configAllSettings(CTREConfigs.armMotorConfig);
    // lowerArmSlave.configAllSettings(CTREConfigs.armMotorConfig);

    // highArmMaster.configAllSettings(CTREConfigs.armMotorConfig);
    // highArmSlave.configAllSettings(CTREConfigs.armMotorConfig);

  
    // lowerArmSlave.setInverted(true);
    highArmSlave.setInverted(true);
  }
  private void waitForHighCanCoder(){
    /*
     * Wait for up to 1000 ms for a good CANcoder signal.
     *
     * This prevents a race condition during program startup
     * where we try to synchronize the Falcon encoder to the
     * CANcoder before we have received any position signal
     * from the CANcoder.
     */
    for (int i = 0; i < 100; ++i) {
      highArmCoder.getAbsolutePosition();
      if (highArmCoder.getLastError() == ErrorCode.OK) {
        break;
      }
      Timer.delay(0.010);            
      HighCANcoderInitTime += 10;
    }
  }
  public void configArmCanCoder() {
    //lowerArmCoder.configFactoryDefault();
    //highArmCoder.configFactoryDefault();
    //lowerArmCoder.configAllSettings(CTREConfigs.armCanCoderConfig);
    //lowerArmCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    //lowerArmCoder.configMagnetOffset(0);
    // highArmCoder.configAllSettings(CTREConfigs.armCanCoderConfig);
    resetToAbsolute();
  }
  public void resetToAbsolute(){
    //waitForLowCanCoder();
    waitForHighCanCoder();

    double highabsolutePosition = Conversions.degreesToFalcon(getHighCanCoder().getDegrees() - highArmAngleOffset, highArmGearRatio);
    highArmMaster.setSelectedSensorPosition(highabsolutePosition);
    highArmSlave.setSelectedSensorPosition(highabsolutePosition);

    // double lowabsolutePosition = Conversions.degreesToFalcon(getLowCanCoder().getDegrees() - lowArmAngleOffset, lowerArmGearRatio);
    // lowerArmMaster.setSelectedSensorPosition(lowabsolutePosition);
    // lowerArmSlave.setSelectedSensorPosition(lowabsolutePosition);
  }
  public Rotation2d getHighCanCoder(){
    return Rotation2d.fromDegrees(highArmCoder.getAbsolutePosition());
  }
  public double getHighAbsoluteAngle(){
    return Conversions.CANcoderToDegrees(highArmCoder.getAbsolutePosition(), highArmGearRatio); 
  }
  public double getHighRelativeAngle(){
    return highArmCoder.getPosition() * (360.0 / (highArmGearRatio * 4096.0));
  }
  public double getHighAngle(){
    return getHighAbsoluteAngle() - highArmAngleOffset;
  }
  public boolean atGoal() {
    return highPidController.atGoal();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    highArmMaster.setVoltage( //TODO want to test flightstick buttons for diff positions 
      highPidController.calculate(getHighRelativeAngle(), 0) //getLowGoal() 
      
      // + 
      // lowArmFeedforward.calculate(
      // lowPidController.getSetpoint().position, 
      // lowPidController.getSetpoint().velocity)
      );
      SmartDashboard.putNumber("High Relative Angle (CANcoder): ", getHighRelativeAngle());
      //SmartDashboard.putNumber("Low Angle from Falcon (Relative Encoder) (test first): ", getHighPosition());
      //SmartDashboard.putNumber("Low Absolute Angle (kinda ignore for now)", getLowAbsoluteAngle());
      //SmartDashboard.putNumber("CANCoder Abs Position", lowerArmCoder.getAbsolutePosition());
      SmartDashboard.putNumber("Motor Volts:", highArmMaster.getMotorOutputVoltage());
  }
}
