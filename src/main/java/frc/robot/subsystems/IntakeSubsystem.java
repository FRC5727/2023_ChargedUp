// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeNeo;

  private boolean cube = true; 
 

  public IntakeSubsystem() {
    intakeNeo = new CANSparkMax(1, MotorType.kBrushless);
  }
  public void setSpeed(double speed) {
    intakeNeo.set(speed);
  }
  public void coneIntake(){
    setSpeed(0.5);
  }
  public void cubeIntake(){
    setSpeed(-0.5);
  }
  public void coneOuttake(){
    setSpeed(-0.5);
  }
  public void cubeOuttake(){
    setSpeed(0.5);
  }
  public void place(){
    if(isCube()) cubeOuttake(); else coneOuttake();
  }
  public void intake(){
    if(isCube()) cubeIntake(); else coneIntake();
  }
  public void cubeIdle(){
    setSpeed(-.12);
  }
  public void coneIdle(){
    setSpeed(0.12);
  }
  public void toggleCube(){
    cube = !cube;
  }
  public boolean isCube(){
    return cube;
  }
  public void intakeIdle(){
    if(isCube()) cubeIdle(); else coneIdle();
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Cube?", cube);
  }
}
