// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeNeo;
  private boolean intakeDebug = false;
  private LED m_led;

  private boolean cube = true;
  private boolean colorInit = false;
  private double currentSpeed = 0.0;
  
  // TODO Move to constants
  private static final double intakeSpeed = 0.50;
  private static final double outtakeSpeed = 0.50;
  private static final double outtakeConeSpeed = 0.35;
  private static final double idleSpeed = 0.18; 
  private static final double stallCurrent = 15.0;
  private static final int stallMaxCube = 10;
  private static final int stallMaxCone = 20;

  private static int stallCounter = 0;

  public IntakeSubsystem(LED s_LED) {
    intakeNeo = new CANSparkMax(1, MotorType.kBrushless);
    
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable sdTable = nt.getTable("SmartDashboard");

    SmartDashboard.putBoolean("Intake debug", intakeDebug);
    sdTable.addListener("Intake debug", EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (table, key, event) -> {
      intakeDebug = event.valueData.value.getBoolean();
    });

    m_led = s_LED;
  }

  public void setSpeed(double speed) {
    currentSpeed = speed;
    intakeNeo.set(speed);
    if (!colorInit) {
      setColor();
    }
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
    if(isCube()) {
      cubeOuttake();
    } else {
      coneOuttake();
    }
  }

  public void intake(){
    if(isCube()) {
      cubeIntake();
    } else {
      coneIntake();
    }
  }
  
  public void cubeIdle(){
    setSpeed(-1.0 * idleSpeed);
  }
  
  public void coneIdle(){
    setSpeed(idleSpeed);
  }
  
  public void toggleCube(){
    cube = !cube;
    setSpeed(-1.0 * currentSpeed);
    setColor();
  }
  
  private void setColor() {
    if (cube) {
      m_led.setColor(LED.Colors.purple);
    } else {
      m_led.setColor(LED.Colors.yellow);
    }
    colorInit = true;
  }
  
  public void disabled() {
    if (colorInit) {
      m_led.setColor(LED.Colors.disabledRed);
      colorInit = false;
    }
  }

  public void flash() {
    m_led.flash(5, LED.Colors.omegabytes);
  }
  
  public boolean isCube(){
    return cube;
  }
  
  public boolean isStalled(){
    return stallCounter >= (cube ? stallMaxCube : stallMaxCone);
  }
  
  public void intakeIdle(){
    if(isCube()) {
      cubeIdle();
    } else {
      coneIdle();
    }
  }
  
  @Override
  public void periodic() {
    if (intakeNeo.getOutputCurrent() > stallCurrent) {
      stallCounter++;
    } else {
      stallCounter = 0;
    }
    SmartDashboard.putBoolean("Cube?", cube);
    if (intakeDebug) {
      SmartDashboard.putNumber("Intake output current", intakeNeo.getOutputCurrent());
      SmartDashboard.putNumber("Intake stall counter", stallCounter);
      SmartDashboard.putBoolean("Piece captured?", isStalled());
    }
  }
}