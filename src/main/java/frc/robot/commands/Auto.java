// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Auto extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public Auto(DriveSubsystem driveSubsystem) {
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset();

    PathPlannerTrajectory test1 = PathPlanner.loadPath("new1", 0.75, 0.75); 
    //install vendor
     
    /* 
    https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json 
    */
    
    addCommands(
      new InstantCommand(() -> driveSubsystem.zeroGyroscope()),
      new WaitCommand(1.0),
      new InstantCommand(() -> driveSubsystem.resetPose(0, 0)),
      new PPSwerveControllerCommand(
        test1,
        driveSubsystem::getPose,
        driveSubsystem.getKinematics(),
        Constants.translationXController,
        Constants.translationYController,
        Constants.rotationController, //Constants.rotationController
        driveSubsystem::setModuleStates,
        driveSubsystem
      ),
      new InstantCommand(() -> driveSubsystem.stop()),
    new WaitCommand(0.5)
      );
  }

  
}
