// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
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

  private ProfiledPIDController lowPidController;
  private ProfiledPIDController highPidController;

  private ArmFeedforward highArmFeedforward;
  private ArmFeedforward lowArmFeedforward;

  private double  L_kp = 0.10,
                  L_ki = 0.00,
                  L_kd = 0.00;

  private double  H_kp = 0.10,
                  H_ki = 0.00,
                  H_kd = 0.00;

  private double  L_ks = 0.00, 
                  L_kg = 0.16, //0.16
                  L_kv = 8.08, //8.08
                  L_ka = 0.03; //0.03
          
  private double  H_ks = 0.00, 
                  H_kg = 0.21, //0.21
                  H_kv = 4.45, //4.45
                  H_ka = 0.02; //0.02

  private double L_maxVelocity, L_maxAcceleration;
  private double H_maxVelocity, H_maxAcceleration;

  private double LowerCANcoderInitTime = 0.0;
  private double HighCANcoderInitTime = 0.0;

  // private double lowGoalDouble;
  // private double highGoalDouble;

  public ArmSubsystem() {
    this.lowerArmMaster = new WPI_TalonFX(Constants.lowerMaster, Constants.rickBot);
    this.lowerArmSlave = new WPI_TalonFX(Constants.lowerSlave, Constants.rickBot);
    this.lowerArmCoder = new CANCoder(4, Constants.rickBot);

    this.highArmMaster = new WPI_TalonFX(Constants.highMaster, Constants.rickBot);
    this.highArmSlave = new WPI_TalonFX(Constants.highSlave, Constants.rickBot);
    this.highArmCoder = new CANCoder(5, Constants.rickBot);

    this.lowPidController = new ProfiledPIDController(L_kp, L_ki, L_kd, 
    new TrapezoidProfile.Constraints(0.025, 0.025));

    this.highPidController = new ProfiledPIDController(H_kp, H_ki, H_kd, 
    new TrapezoidProfile.Constraints(0.025, 0.025));

    this.lowArmFeedforward = new ArmFeedforward(L_ks, L_kg, L_kv, L_ka);
    this.highArmFeedforward = new ArmFeedforward(H_ks, H_kg, H_kv, H_ka);

    this.lowArmAngleOffset = 0;
    this.highArmAngleOffset = 287.92; 

    this.lowerArmGearRatio = 0.40; //4.125
    this.highArmGearRatio = 0.10; 

    this.L_maxVelocity = 0.025; // Degrees per second
    this.L_maxAcceleration = 0.025; // Degrees per second squared
    
    this.H_maxVelocity = 0.025; // Degrees per second 
    this.H_maxAcceleration = 0.025; // Degrees per second squared
    // lowPidController.disableContinuousInput();
    // highPidController.disableContinuousInput();
    
    // configArmMotor();
    // resetToAbsolute();
    // configArmCanCoder();
    // lowerArmSlave.follow(lowerArmMaster);
    // highArmSlave.follow(highArmMaster);
    
    // lowerArmSlave.setInverted(false);
    // lowerArmMaster.setInverted(true);

    // highArmSlave.setInverted(true);

    //lowerArmCoder.setPosition(0);
    lowPidController.disableContinuousInput();
    lowPidController.setTolerance(5, 5);
    
    highPidController.disableContinuousInput();
    highPidController.setTolerance(5, 5);
    // lowGoalDouble = 0;
    // highGoalDouble = 0;
  }
  public void configArmMotor(){
    lowerArmSlave.follow(lowerArmMaster);
    highArmSlave.follow(highArmMaster);

    lowerArmMaster.configFactoryDefault();
    lowerArmSlave.configFactoryDefault();

    highArmMaster.configFactoryDefault();
    highArmSlave.configFactoryDefault();

    // lowerArmMaster.configAllSettings(CTREConfigs.armMotorConfig);
    // lowerArmSlave.configAllSettings(CTREConfigs.armMotorConfig);

    // highArmMaster.configAllSettings(CTREConfigs.armMotorConfig);
    // highArmSlave.configAllSettings(CTREConfigs.armMotorConfig);

  
    lowerArmSlave.setInverted(true);
    highArmSlave.setInverted(true);
  }
  private void waitForLowCanCoder(){
    /*
     * Wait for up to 1000 ms for a good CANcoder signal.
     *
     * This prevents a race condition during program startup
     * where we try to synchronize the Falcon encoder to the
     * CANcoder before we have received any position signal
     * from the CANcoder.
     */
    for (int i = 0; i < 100; ++i) {
      lowerArmCoder.getAbsolutePosition();
      if (lowerArmCoder.getLastError() == ErrorCode.OK) {
        break;
      }
      Timer.delay(0.010);            
      LowerCANcoderInitTime += 10;
    }
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
  // public void configArmCanCoder() {
  //   //lowerArmCoder.configFactoryDefault();
  //   //highArmCoder.configFactoryDefault();
  //   //lowerArmCoder.configAllSettings(CTREConfigs.armCanCoderConfig);
  //   //lowerArmCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
  //   //lowerArmCoder.configMagnetOffset(0);
  //   // highArmCoder.configAllSettings(CTREConfigs.armCanCoderConfig);
  //   resetToAbsolute();
  // }
  // public void resetToAbsolute(){
  //   waitForLowCanCoder();
  //   waitForHighCanCoder();

  //   double highabsolutePosition = Conversions.degreesToFalcon(getHighCanCoder().getDegrees() - highArmAngleOffset, highArmGearRatio);
  //   highArmMaster.setSelectedSensorPosition(highabsolutePosition);
  //   highArmSlave.setSelectedSensorPosition(highabsolutePosition);

  //   double lowabsolutePosition = Conversions.degreesToFalcon(getLowCanCoder().getDegrees() - lowArmAngleOffset, lowerArmGearRatio);
  //   lowerArmMaster.setSelectedSensorPosition(lowabsolutePosition);
  //   lowerArmSlave.setSelectedSensorPosition(lowabsolutePosition);
  // }

  public Rotation2d getHighCanCoder(){
    return Rotation2d.fromDegrees(highArmCoder.getAbsolutePosition());
  }
  public Rotation2d getLowCanCoder(){
    return Rotation2d.fromDegrees(highArmCoder.getAbsolutePosition());
  }

  public double getHighAbsoluteAngle(){
    return Conversions.CANcoderToDegrees(highArmCoder.getAbsolutePosition(), highArmGearRatio); 
  }
  public double getLowAbsoluteAngle(){
    return Conversions.CANcoderToDegrees(lowerArmCoder.getAbsolutePosition(), lowerArmGearRatio); 
  } 
  
  public double getLowRelativeAngle(){
    return lowerArmCoder.getPosition() * (360.0 / (lowerArmGearRatio * 4096.0));
  }
  public double getHighRelativeAngle(){
    return highArmCoder.getPosition() * (360.0 / (highArmGearRatio * 4096.0));
  }

  // public double getLowPosition(){
  //   return Conversions.falconToDegrees(lowerArmMaster.getSelectedSensorPosition(), lowerArmGearRatio);
  // }
  // public double getHighPosition(){
  //   return Conversions.falconToDegrees(highArmMaster.getSelectedSensorPosition(), highArmGearRatio);
  // }

  public double getHighAngle(){
    return getHighAbsoluteAngle() - highArmAngleOffset;
  }
  public double getLowAngle(){
    return getLowAbsoluteAngle() - lowArmAngleOffset;
  }

  // public void setHighArmGoal(double angle){
  //   highPidController.setGoal(angle);
  // }
  // public void setLowArmGoal(double angle){
  //   lowPidController.setGoal(angle);
  // }

  // public void chassisPos(){
  //   lowGoalDouble = 0;
  //   highGoalDouble = 0;
  // }
  // public void intakeGroundPos(){
  //   lowGoalDouble = 36;
  //   highGoalDouble = -29;
  // }
  // public void highPos(){
  //   lowGoalDouble = 54;
  //   highGoalDouble = 86;
  // }
  // public void midPos(){
  //   lowGoalDouble = 26;
  //   highGoalDouble = 121;
  // }
  // public void lowPos(){
  //   lowGoalDouble = 27;
  //   highGoalDouble = 2;
  // }

  // public double getHighGoal(){
  //   return highPidController.getGoal().position;
  // }
  // public double getLowGoal(){
  //   return lowPidController.getGoal().position;
  // }
  // public void doTheThing(){
  //   lowGoalDouble = getLowGoal();
  //   highGoalDouble = getHighGoal();
  // }
  @Override
  public void periodic() {
    //doTheThing();
    // This method will be called once per scheduler run
      // highArmMaster.setVoltage( //sets the voltage according to the PID controller and armFeedForward
      highPidController.calculate(getHighRelativeAngle(), 86) //getHighGoal()
      // + //PID calculates according to its current angle, and the goal angle its trying to get to
      // highArmFeedforward.calculate(
      // highPidController.getSetpoint().position, 
      // highPidController.getSetpoint().velocity)
      ;
    
      lowerArmMaster.setVoltage(
      lowPidController.calculate(getLowRelativeAngle(), 54) //getLowGoal() 
      
      // // + 
      // // lowArmFeedforward.calculate(
      // // lowPidController.getSetpoint().position, 
      // // lowPidController.getSetpoint().velocity)
      );
      SmartDashboard.putNumber("Low Relative Angle (CANcoder): ", getLowRelativeAngle());
      SmartDashboard.putNumber("High Relative Angle (CANcoder): ", getHighRelativeAngle());
      // SmartDashboard.putNumber("number", lowGoalDouble);
      // SmartDashboard.putNumber("number2", highGoalDouble);
      // SmartDashboard.putNumber("High Arm Master Output Voltage", highArmMaster.getMotorOutputVoltage());
      // SmartDashboard.putNumber("High Arm Angle: ", getHighAngle());
      // SmartDashboard.putNumber("Low Arm Angle: ", getLowAngle());
      // SmartDashboard.putNumber("Arm Low CanCoder POS", getLowPosition());
      // SmartDashboard.putNumber("Arm Low Master Pos", Conversions.falconToDegrees(lowerArmMaster.getSelectedSensorPosition(), lowerArmGearRatio));
      // SmartDashboard.putNumber("Arm Low Arm Slave Pos", Conversions.falconToDegrees(lowerArmSlave.getSelectedSensorPosition(), lowerArmGearRatio));
      // SmartDashboard.putNumber("volts", lowerArmMaster.getMotorOutputVoltage());
      // SmartDashboard.putNumber("THIS", getLowAbsoluteAngle());
      // SmartDashboard.putNumber("LOOK AT THIS", getLowRelativeAngle());
  }
}
