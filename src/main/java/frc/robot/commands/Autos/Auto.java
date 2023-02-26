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

public class Auto extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public Auto(Swerve s_Swerve) {
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset();

    PathPlannerTrajectory test1 = PathPlanner.loadPath("new1", 1.00, 1.00); 
    //install vendor
     
    /* 
    https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json 
    */
    
    addCommands(
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      new WaitCommand(1.0),
      // new InstantCommand(() -> s_Swerve.resetPose(0, 0)),
      new PPSwerveControllerCommand(
        test1,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        Constants.translationXController,
        Constants.translationYController,
        Constants.rotationController, //Constants.rotationController
        s_Swerve::setModuleStates,
        s_Swerve
      ),
      new InstantCommand(() -> s_Swerve.stop()),
      new WaitCommand(0.5)
      );
  }

  
}
