// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimerSubsystem extends SubsystemBase {
  /** Creates a new TimerSubsystem. */
  public double gameTime;
  public double gameTimeConversion;
  public Timer timer;
  public TimerSubsystem() { 
    timer = new Timer();
  }

  @Override
  public void periodic() {
    gameTime = Timer.getMatchTime();
    SmartDashboard.putNumber("Match Timer", gameTime);
  }
}
