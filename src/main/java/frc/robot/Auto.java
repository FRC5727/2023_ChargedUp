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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.Position;

public class Auto {
    private static final double autoMaxVel = 3.0;
    private static final double autoMaxAccel = 1.0;

    private HashMap<String, Command> eventMap = new HashMap<>();
    private SwerveAutoBuilder autoBuilder;

    private final SendableChooser<Boolean> pieceChooser = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser2 = new SendableChooser<>();

    private Command toggleCommand(IntakeSubsystem s_Intake) {
        return Commands.runOnce(() -> { if (!pieceChooser.getSelected()) s_Intake.toggleCube(); });
    }

    private Command placePieceCommand(IntakeSubsystem s_Intake, ArmSubsystem s_Arm, Position position)
    {
        return new IdleCommand(s_Intake)
            .raceWith(
                Commands.runOnce(() -> s_Arm.setTargetPosition(position))
                    .andThen(new ArmCommand(s_Arm).withTimeout(7.0)))
            .andThen(justPlaceCommand(s_Intake));
    }

    private Command justPlaceCommand(IntakeSubsystem s_Intake)
    {
        return new PlaceCommand(s_Intake).withTimeout(1.0)
            .andThen(new IdleCommand(s_Intake));
    }
  
    public Auto(ArmSubsystem s_Arm, IntakeSubsystem s_Intake, Swerve s_Swerve) {
        eventMap.put("Place cube low", new PlaceCommand(s_Intake));
        eventMap.put("Place cube high",
            new IdleCommand(s_Intake)
                .raceWith(new ArmCommand(s_Arm, Position.GRID_HIGH).withTimeout(7.0))
            .andThen(new PlaceCommand(s_Intake).withTimeout(2.0))
            .andThen(new IdleCommand(s_Intake)
                .raceWith(new ArmCommand(s_Arm, Position.CHASSIS))));
        eventMap.put("Place piece", toggleCommand(s_Intake).andThen(placePieceCommand(s_Intake, s_Arm, placeChooser.getSelected())));
        eventMap.put("Place cube", placePieceCommand(s_Intake, s_Arm, placeChooser.getSelected()));
        eventMap.put("Place second cube", justPlaceCommand(s_Intake));
        eventMap.put("Prepare to place second piece", new ArmCommand(s_Arm, placeChooser2.getSelected()));
        eventMap.put("Chassis", new ArmCommand(s_Arm, Position.CHASSIS));
        eventMap.put("Ground intake", new IntakeCommand(s_Intake).alongWith(new ArmCommand(s_Arm, Position.INTAKE_GROUND)));
        eventMap.put("Balance", new PrintCommand("TODO Auto-balance"));

        autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
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
        placeChooser.setDefaultOption("Chassis", Position.CHASSIS);
        SmartDashboard.putData("First placement location", placeChooser);

        placeChooser2.addOption("High", Position.GRID_HIGH);
        placeChooser2.addOption("Middle", Position.GRID_MID);
        placeChooser2.addOption("Low", Position.GRID_LOW);
        placeChooser2.setDefaultOption("Chassis", Position.CHASSIS);
        SmartDashboard.putData("Second placement location", placeChooser);
    }

    public Command buildCommand(String pathName) {
        if (pathName == null)
            return null;

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, autoMaxVel, autoMaxAccel);

        return autoBuilder.fullAuto(pathGroup);
    }

    public static List<String> getPathnames() {
        return Stream.of(new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles())
                .filter(file -> !file.isDirectory())
                .filter(file -> file.getName().matches("\\.path$"))
                .map(File::getName)
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
    }
}