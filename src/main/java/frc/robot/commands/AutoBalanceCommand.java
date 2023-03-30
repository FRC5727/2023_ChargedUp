// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  private final Swerve s_Swerve;
  private final LED s_LED;
  private final PIDController m_PID = new PIDController(0.02, 0, 0);
  private final Timer timer = new Timer();
  private final double angleTarget = 2.5;
  private final double angleMax = 15.0;

  private boolean usingPID = false;
  private double switchPoint = 8;
  private boolean paused = false;
  private double pauseDelay = 0.25;

  public AutoBalanceCommand(Swerve s_Swerve, LED s_LED) {
    this.s_Swerve = s_Swerve;
    this.s_LED = s_LED;
    addRequirements(s_Swerve);

    m_PID.setTolerance(angleTarget);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double pitch = s_Swerve.getPitch();
    double speed = Math.signum(pitch) * -0.5; // default speed (m/s) if more than switchPoint

    if (!usingPID && Math.abs(pitch) < switchPoint) {
      usingPID = true;
      m_PID.reset();
      paused = true;
      timer.start();
    }
    if (usingPID) {
      speed = m_PID.calculate(pitch, 0);
    }

    if (timer.hasElapsed(pauseDelay)) {
      timer.stop();
      paused = false;
    }

    boolean balanced = usingPID ? m_PID.atSetpoint() : false;
    Translation2d driving = new Translation2d((balanced || paused) ? 0 : speed, 0);
    SmartDashboard.putBoolean("Autobalanced", balanced);
    SmartDashboard.putNumber("Autobalance speed", speed);
    SmartDashboard.putBoolean("Autobalance PID", usingPID);
    SmartDashboard.putBoolean("Autobalanced paused", paused);
    if (balanced) {
      s_LED.setColor(LED.Colors.green);
      s_LED.setStrobe(LED.Colors.green, 100);
    } else {
      double colorPercent = Math.max(Math.min((angleMax - Math.abs(pitch)) / (angleMax - angleTarget) * 100, 100), 0) * 0.9 + 10;

      s_LED.setColor(LED.Colors.off, colorPercent, 100);
      if (usingPID) {
        s_LED.setColor(LED.Colors.cone, 0, colorPercent);
      } else {
        s_LED.setColor(LED.Colors.blue, 0, colorPercent);
      }
    }
    s_Swerve.drive(driving, 0, false, false);
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stopDrive();
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}