// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.RED;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PlaceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ArmSubsystem.Position;

public class REDHighCubeChargeStation extends SequentialCommandGroup {
  /** Creates a new ChargeStationRedSideAuto. */
  public REDHighCubeChargeStation(Swerve s_Swerve, ArmCommand armCommand, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, IntakeCommand intakeCommand, PlaceCommand placeCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    addCommands(armCommand);
    addCommands(intakeCommand);
    addCommands(placeCommand);
    addRequirements(intakeSubsystem);
    addRequirements(armSubsystem);
    
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset();
    PathPlannerTrajectory path = PathPlanner.loadPath("ChargeStationRedCenter1", 1.75, 1.75);
    //ChargeStationRedCenter1
    addCommands(
      new InstantCommand(() -> armSubsystem.setTargetPosition(Position.MIDDLE)),
      new InstantCommand(() -> new ArmCommand(armSubsystem)),
      new WaitCommand(2),
      new InstantCommand(() -> armSubsystem.setTargetPosition(Position.GRID_HIGH)),
      new InstantCommand(() -> new ArmCommand(armSubsystem)),
      new WaitCommand(2),
      new InstantCommand(() -> new PlaceCommand(intakeSubsystem)),
      new WaitCommand(1),
      new InstantCommand(() -> armSubsystem.setTargetPosition(Position.CHASSIS)),
      new InstantCommand(() -> new ArmCommand(armSubsystem)),
      new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(14.72, 2.75), new Rotation2d(0)))),
      new WaitCommand(1.0),
      new InstantCommand(() -> s_Swerve.getPose()),
      new PPSwerveControllerCommand(
        path,
        s_Swerve::getPose, //it figures out where it is at
        Constants.Swerve.swerveKinematics, //gets the kinematics
        new PIDController(0.7, 0, 0), //X
        new PIDController(0.7, 0, 0), //Y
        new PIDController(0, 0, 0), //Rotation
        // Constants.translationXController, //X Movement PID controller
        // Constants.translationYController, //Y Movement PID controller
        // Constants.rotationController, //Rotation PID controller
        s_Swerve::setModuleStates, //makes the swerve move according to the path
        s_Swerve //it needs this so it can actually drive
      ),
      new InstantCommand(() -> s_Swerve.stop()),
      new WaitCommand(0.5)
    );
  }
}
