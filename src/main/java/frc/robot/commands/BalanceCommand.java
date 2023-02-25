// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class BalanceCommand extends CommandBase {
  /** Creates a new BalanceCommand. */
  private double m_tolerance = 3.5; // degrees
    private final PIDController m_balancePID = new PIDController(-0.025, 0, 0);

    private Swerve m_driveSubsystem;

    private double m_pitchAngle = 0;

    public BalanceCommand(Swerve s_Swerve) {
        m_driveSubsystem = s_Swerve;
        addRequirements(m_driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_balancePID.enableContinuousInput(-180, 180);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // TODO Develop implementation
      // m_pitchAngle = m_driveSubsystem.getGyroPitch();
        
      // if (Math.abs(m_pitchAngle) > m_tolerance)
      //   m_driveSubsystem.driveAuto(m_balancePID.calculate(m_pitchAngle, 0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

