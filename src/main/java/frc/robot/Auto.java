package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.Position;

public class Auto {
    private HashMap<String, Command> eventMap = new HashMap<>();
    private SwerveAutoBuilder autoBuilder;

    private final SendableChooser<Boolean> pieceChooser = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser2 = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser3 = new SendableChooser<>();

    private Command justPlaceCommand(IntakeSubsystem s_Intake)
    {
        return new PlaceCommand(s_Intake).withTimeout(0.2);
    }
  
    public Auto(ArmSubsystem s_Arm, IntakeSubsystem s_Intake, Swerve s_Swerve) {
        eventMap.put("Place first piece", 
            Commands.runOnce(() -> { if (!pieceChooser.getSelected().booleanValue()) s_Intake.toggleCube(); })
                .andThen(new ArmCommand(s_Arm, placeChooser::getSelected)
                    .raceWith(new IdleCommand(s_Intake)))
                .andThen(justPlaceCommand(s_Intake)));
        eventMap.put("Move arm to second", new ArmCommand(s_Arm, placeChooser2::getSelected));
        eventMap.put("Move arm to third", new ArmCommand(s_Arm, placeChooser3::getSelected));
        eventMap.put("Eject cube", justPlaceCommand(s_Intake));
        eventMap.put("Chassis", new ArmCommand(s_Arm, Position.CHASSIS));
        eventMap.put("Ground intake",
            new ArmCommand(s_Arm, Position.INTAKE_GROUND)
                .alongWith(
                    Commands.runOnce(() -> { if (!s_Intake.isCube()) s_Intake.toggleCube(); })
                    .andThen(new WaitCommand(1.0))
                    .andThen(new IntakeCommand(s_Intake))));
        eventMap.put("Balance", new AutoBalanceCommand(s_Swerve));

        autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(10, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(2.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Mirror Blue path to Red automatically
            s_Swerve);

        pieceChooser.setDefaultOption("Cube", Boolean.TRUE);
        pieceChooser.addOption("Cone", Boolean.FALSE);
        SmartDashboard.putData("Starting game piece", pieceChooser);

        placeChooser.addOption("High", Position.GRID_HIGH);
        placeChooser.addOption("Middle", Position.GRID_MID);
        placeChooser.addOption("Low", Position.GRID_LOW);
        placeChooser.addOption("Chassis", Position.CHASSIS);
        placeChooser.setDefaultOption("No movement", Position.NONE);
        SmartDashboard.putData("First placement location", placeChooser);

        placeChooser2.addOption("High (2)", Position.GRID_HIGH);
        placeChooser2.addOption("Middle", Position.GRID_MID);
        placeChooser2.addOption("Low", Position.GRID_LOW);
        placeChooser2.setDefaultOption("Chassis", Position.CHASSIS);
        SmartDashboard.putData("Second placement location", placeChooser2);
        
        placeChooser3.addOption("High (3)", Position.GRID_HIGH);
        placeChooser3.addOption("Middle", Position.GRID_MID);
        placeChooser3.addOption("Low", Position.GRID_LOW);
        placeChooser3.setDefaultOption("Chassis", Position.CHASSIS);
        SmartDashboard.putData("Third placement location", placeChooser3);
    }

    public Command buildCommand(String pathName) {
        if (pathName == null)
            return null;

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, PathPlanner.getConstraintsFromPath(pathName));
        return autoBuilder.fullAuto(pathGroup);
    }

    public static List<String> getPathnames() {
        return Stream.of(new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles())
                .filter(file -> !file.isDirectory())
                .filter(file -> file.getName().matches(".*\\.path"))
                .map(File::getName)
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
    }
}