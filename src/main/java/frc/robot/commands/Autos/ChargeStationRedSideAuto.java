// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ChargeStationRedSideAuto extends SequentialCommandGroup {
  /** Creates a new ChargeStationRedSideAuto. */
  public ChargeStationRedSideAuto(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    
      
     
      
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset();
    PathPlannerTrajectory a_ChargeStationRedCenter1 = PathPlanner.loadPath("ChargeStationRedCenter1", 0.5, 0.5);
    
    addCommands(
      new WaitCommand(1.0),
      new InstantCommand(() -> driveSubsystem.resetPose(14.13, 2.75)),
      // new InstantCommand(() -> driveSubsystem.getPose()),
      new PPSwerveControllerCommand(
        a_ChargeStationRedCenter1,
        driveSubsystem::getPose, //it figures out where it is at
        driveSubsystem.getKinematics(), //gets the kinematics
        new PIDController(0.7, 0, 0), //X
        new PIDController(0.7, 0, 0), //Y
        new PIDController(0, 0, 0), //Rotation
        // Constants.translationXController, //X Movement PID controller
        // Constants.translationYController, //Y Movement PID controller
        // Constants.rotationController, //Rotation PID controller
        driveSubsystem::setModuleStates, //makes the swerve move according to the path
        driveSubsystem //it needs this so it can actually drive
      ),
      new InstantCommand(() -> driveSubsystem.stop()),
      new WaitCommand(0.5)
    );
  }
}
