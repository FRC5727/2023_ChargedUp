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
  private final LEDSubsystem m_led = new LEDSubsystem();

  private boolean cube = true;

  // TODO Move to constants
  private static final double intakeSpeed = 0.50;
  private static final double outtakeSpeed = 0.50;
  private static final double outtakeConeSpeed = 0.35;
  private static final double idleSpeed = 0.18; 
  private static final double stallCurrent = 15.0;

  public IntakeSubsystem() {
    intakeNeo = new CANSparkMax(1, MotorType.kBrushless);

    m_led.setRainbow(); 
  
  }
  public void setSpeed(double speed) {
    intakeNeo.set(speed);
  }
  public void coneIntake(){
    setSpeed(intakeSpeed);
  }
  public void cubeIntake(){
    setSpeed(-1.0 * intakeSpeed);
  }
  public void coneOuttake(){
    setSpeed(-1.0 * outtakeConeSpeed);;
  }
  public void cubeOuttake(){
    setSpeed(outtakeSpeed);
  }
  public void place(){
    if(isCube()) cubeOuttake(); else coneOuttake();
  }
  public void intake(){
    if(isCube()) cubeIntake(); else coneIntake();
  }
  public void cubeIdle(){
    setSpeed(-1.0 * idleSpeed);
  }
  public void coneIdle(){
    setSpeed(idleSpeed);
  }
  public void toggleCube(){
    cube = !cube;
  
     if(cube){
      //Set color to purple
         m_led.setColor(186, 0, 255);
     }
     else {
         m_led.setColor(255,191,0);
     }
  
  }
  public boolean isCube(){
    return cube;
  }
  public boolean isStalled(){
    return intakeNeo.getOutputCurrent() > stallCurrent;
  }
  public void intakeIdle(){
    if(isCube()) cubeIdle(); else coneIdle();
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Cube?", cube);
    SmartDashboard.putNumber("Intake output current", intakeNeo.getOutputCurrent());
    SmartDashboard.putBoolean("Piece captured?", isStalled());
  }
}