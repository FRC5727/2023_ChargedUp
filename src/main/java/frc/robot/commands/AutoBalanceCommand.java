// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  private final Swerve m_subsystem;
  private final PIDController m_PID = new PIDController(0.035, 0, 0);
  private final int count = 0;
  public AutoBalanceCommand(Swerve subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_PID.setTolerance(5.0, .5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double directionforce = m_PID.calculate(m_subsystem.getPitch(), 0);
    Translation2d driving = new Translation2d(m_PID.atSetpoint() ? 0 : directionforce, 0);
    SmartDashboard.putNumber("AB force", directionforce);
    m_subsystem.drive(driving, 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
