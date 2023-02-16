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
import frc.robot.Constants;

public class LowArmSubsystem extends SubsystemBase {
  /** Creates a new LowArmSubsystem. */
  // private WPI_TalonFX lowerArmMaster;
  // private WPI_TalonFX lowerArmSlave;

  // private CANCoder lowerArmCoder;
  
  // private double lowArmAngleOffset;

  // private double lowerArmGearRatio;

  // private ProfiledPIDController lowPidController;
  // private ArmFeedforward lowArmFeedforward;

  // private double  L_kp = 0.10,
  //                 L_ki = 0.00,
  //                 L_kd = 0.00;
                 
  // private double  L_ks = 0.00, 
  //                 L_kg = 0.16, //0.16
  //                 L_kv = 8.08, //8.08
  //                 L_ka = 0.03; //0.03


  // private double L_maxVelocity, L_maxAcceleration;

  // private double LowerCANcoderInitTime = 0.0;

  public LowArmSubsystem() {
    // this.lowerArmMaster = new WPI_TalonFX(Constants.lowerMaster, Constants.rickBot);
    // this.lowerArmSlave = new WPI_TalonFX(Constants.lowerSlave, Constants.rickBot);
    // this.lowerArmCoder = new CANCoder(4, Constants.rickBot);
    
    // this.lowPidController = new ProfiledPIDController(L_kp, L_ki, L_kd, 
    // new TrapezoidProfile.Constraints(0.025, 0.025));

    // this.lowArmFeedforward = new ArmFeedforward(L_ks, L_kg, L_kv, L_ka);

    // this.lowArmAngleOffset = 0;

    // this.lowerArmGearRatio = 0.40; //4.125

    // // this.L_maxVelocity = 0.025; // Degrees per second
    // // this.L_maxAcceleration = 0.025; // Degrees per second squared

    // configArmMotor();
    // resetToAbsolute();
    // configArmCanCoder();

    // lowPidController.disableContinuousInput();
    // lowPidController.setTolerance(5, 5);

  }
  // public void configArmMotor(){
  //   lowerArmSlave.follow(lowerArmMaster);
  //   lowerArmMaster.configFactoryDefault();
  //   lowerArmSlave.configFactoryDefault();
  //   lowerArmSlave.setInverted(true);
  //   //lowPidController.setTolerance(5, 5);
  // }
  // private void waitForLowCanCoder(){
  //   /*
  //    * Wait for up to 1000 ms for a good CANcoder signal.
  //    *
  //    * This prevents a race condition during program startup
  //    * where we try to synchronize the Falcon encoder to the
  //    * CANcoder before we have received any position signal
  //    * from the CANcoder.
  //    */
  //   for (int i = 0; i < 100; ++i) {
  //     lowerArmCoder.getAbsolutePosition();
  //     if (lowerArmCoder.getLastError() == ErrorCode.OK) {
  //       break;
  //     }
  //     Timer.delay(0.010);            
  //     LowerCANcoderInitTime += 10;
  //   }
  // }
  // public void configArmCanCoder() {
  //   lowerArmCoder.configFactoryDefault();
  //   //highArmCoder.configFactoryDefault();
  //   //lowerArmCoder.configAllSettings(CTREConfigs.armCanCoderConfig);
  //   // lowerArmCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
  //   //lowerArmCoder.configMagnetOffset(0);
  //   // highArmCoder.configAllSettings(CTREConfigs.armCanCoderConfig);
  //   resetToAbsolute();
  // }
  // public void resetToAbsolute(){
  //   waitForLowCanCoder();
  //   double lowabsolutePosition = Conversions.degreesToFalcon(getLowCanCoder().getDegrees() - lowArmAngleOffset, lowerArmGearRatio);
  //   lowerArmMaster.setSelectedSensorPosition(lowabsolutePosition);
  //   lowerArmSlave.setSelectedSensorPosition(lowabsolutePosition);
  // }
  // public Rotation2d getLowCanCoder(){
  //   return Rotation2d.fromDegrees(lowerArmCoder.getAbsolutePosition());
  // }
  // public double getLowAbsoluteAngle(){
  //   return Conversions.CANcoderToDegrees(lowerArmCoder.getAbsolutePosition(), lowerArmGearRatio); 
  // } 
  // public double getLowRelativeAngle(){
  //   return lowerArmCoder.getPosition() * (360.0 / (lowerArmGearRatio * 4096.0));
  // }
  // public double getLowAngle(){
  //   return getLowAbsoluteAngle() - lowArmAngleOffset;
  // }
  // public void setLowArmGoal(double angle){
  //   lowPidController.setGoal(25);
  // }
  // public double getLowGoal(){
  //   return lowPidController.getGoal().position;
  // }
  // public boolean atGoal() {
  //   return lowPidController.atGoal();
  // }
  // public double getLowPosition() {
  //   return Conversions.falconToDegrees(lowerArmMaster.getSelectedSensorPosition(), lowerArmGearRatio);
  // }



  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  //   // lowerArmMaster.setVoltage( //TODO want to test flightstick buttons for diff positions 
  //   //   lowPidController.calculate(getLowRelativeAngle(), 0) //getLowGoal() 
      
  //   //   // + 
  //   //   // lowArmFeedforward.calculate(
  //   //   // lowPidController.getSetpoint().position, 
  //   //   // lowPidController.getSetpoint().velocity)
  //   //   );
  //   //   SmartDashboard.putNumber("Low Relative Angle (CANcoder): ", getLowRelativeAngle());
  //   //   SmartDashboard.putNumber("Low Angle from Falcon (Relative Encoder) (test first): ", getLowPosition());
  //   //   SmartDashboard.putNumber("Low Absolute Angle (kinda ignore for now)", getLowAbsoluteAngle());
  //   //   SmartDashboard.putNumber("CANCoder Abs Position", lowerArmCoder.getAbsolutePosition());
  //   //   SmartDashboard.putNumber("Motor Volts:", lowerArmMaster.getMotorOutputVoltage());
  // }
}
