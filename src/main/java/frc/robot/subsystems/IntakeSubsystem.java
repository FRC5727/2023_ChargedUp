// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  //private final CANSparkMax intakeNeo;
  
 

  public IntakeSubsystem() {
    //intakeNeo = new CANSparkMax(0, MotorType.kBrushless);
  }
  public void setSpeed(double speed) {
    //intakeNeo.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(intakeNeo.getOutputCurrent() > 5){
    //   setSpeed(0);
    // }
  }
}
