// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class StraightLineRedToCargo4Auto extends SequentialCommandGroup {
  /** Creates a new StraightLineRedToCargo4Auto. */
  public StraightLineRedToCargo4Auto(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset();

    PathPlannerTrajectory a_straightLineRedToCargo4 = PathPlanner.loadPath("StraightLineRedToCargo4", 1.00, 1.00);

    addCommands(
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      new WaitCommand(1.0),
      // new InstantCommand(() -> driveSubsystem.resetPose(0, 0)),
      new PPSwerveControllerCommand(
        a_straightLineRedToCargo4,
        s_Swerve::getPose, // it figures out where it is at
        Constants.Swerve.swerveKinematics, // gets the kinematics
        Constants.translationXController, // X Movement PID controller
        Constants.translationYController, // Y Movement PID controller
        Constants.rotationController, // Rotation PID controller
        s_Swerve::setModuleStates, // makes the swerve move according to the path
        s_Swerve // it needs this so it can actually drive
      ),
      new InstantCommand(() -> s_Swerve.stop()),
      new WaitCommand(0.5));
  }
}