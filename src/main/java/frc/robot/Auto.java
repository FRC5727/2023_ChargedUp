package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
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
    private final HashMap<String, Command> eventMap = new HashMap<>();
    private final SwerveAutoBuilder autoBuilder;

    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> pieceChooser = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser2 = new SendableChooser<>();
    private final SendableChooser<ArmSubsystem.Position> placeChooser3 = new SendableChooser<>();
    private final SendableChooser<Command> quickChooser = new SendableChooser<>();

    private final double firstArmTimeout = 3.25;
    private AutoConfig activeConfig;

    public static class AutoConfig {
        private final String path;
        private final boolean piece;
        private final ArmSubsystem.Position place1;
        private final ArmSubsystem.Position place2;
        private final ArmSubsystem.Position place3;

        public AutoConfig(String path, boolean piece, ArmSubsystem.Position place1,
                          ArmSubsystem.Position place2, ArmSubsystem.Position place3) {
            this.path = path;
            this.piece = piece;
            this.place1 = place1;
            this.place2 = place2;
            this.place3 = place3;
        }
    }

    private final SortedMap<String, AutoConfig> quickpicks = new TreeMap<>(Map.ofEntries(
        Map.entry("[No bump] 3 cube link [innermost]", new AutoConfig("SS-Place link (new)", true, Position.NONE, Position.CHASSIS, Position.CHASSIS)),
        Map.entry("[No bump] 2 high + balance [outer]", new AutoConfig("SS-PlaceCone,pick,place,balance", false, Position.GRID_HIGH, Position.GRID_HIGH, Position.NONE)),
        Map.entry("[No bump] 2.5 w/2 high [outer]", new AutoConfig("SS-PlaceCone,pick,place,pick", false, Position.GRID_HIGH, Position.GRID_HIGH, Position.NONE)),
        Map.entry("[No bump-BLUE] 3 cube mixed [align inside edge BLUE]", new AutoConfig("SS-Place three mixed", true, Position.NONE, Position.GRID_HIGH, Position.CHASSIS)),
        Map.entry("[No bump-RED] 3 cube mixed [align inside edge RED]", new AutoConfig("SSRed-Place three mixed", true, Position.NONE, Position.GRID_HIGH, Position.CHASSIS)),
        Map.entry("[Middle cone] 1 high, over, back and balance", new AutoConfig("Mid-Place,mobility,balance", false, Position.GRID_HIGH, Position.NONE, Position.NONE)),
        Map.entry("[Middle cube] 1 high", new AutoConfig("Mid-Place only", false, Position.GRID_HIGH, Position.NONE, Position.NONE)),
        Map.entry("[Middle cube] 1 high, delayed mobility", new AutoConfig("Mid-Place, delayed mobility", false, Position.GRID_HIGH, Position.NONE, Position.NONE)),
        Map.entry("[Speed bump] 1.5 high + balance", new AutoConfig("SB-PlaceCone,pick,balance", false, Position.GRID_HIGH, Position.NONE, Position.NONE)),
        Map.entry("[Speed bump] 2 high [outer]", new AutoConfig("SB-PlaceCone,pick,place,pick", false, Position.GRID_HIGH, Position.GRID_HIGH, Position.NONE))
        // Map.entry("2.5 w/2 high -- Yoshi mode [substation, inner]", new AutoConfig("SSInside-PlaceCone,Yoshi,place,Yoshi,return", false, Position.GRID_HIGH, Position.GRID_HIGH, Position.NONE)),
        // Map.entry("[Speed bump] 1.5 high + balance [cube]", new AutoConfig("SB-Place,pick,balance", true, Position.GRID_HIGH, Position.NONE, Position.NONE)),
        // Map.entry("2 low + high [speed bump, cube]", new AutoConfig("SB-Place,pick,place,position", true, Position.CHASSIS, Position.GRID_HIGH, Position.NONE))
    ));

    private Command justPlaceCommand(IntakeSubsystem s_Intake)
    {
        return Commands.startEnd(s_Intake::place, () -> {}, s_Intake).withTimeout(0.2);
    }

    public Auto(ArmSubsystem s_Arm, IntakeSubsystem s_Intake, Swerve s_Swerve, LED s_LED) {
        eventMap.put("Place first piece",
            Commands.runOnce(() -> { if (activeConfig.piece != s_Intake.isCube()) s_Intake.toggleCube(); })
                .andThen(Commands.runOnce(s_Intake::idle, s_Intake))
                .andThen((new ArmCommand(s_Arm, () -> activeConfig.place1))
                    .withTimeout(firstArmTimeout))
                .andThen(justPlaceCommand(s_Intake)));
        eventMap.put("Move arm to second", new ArmCommand(s_Arm, () -> activeConfig.place2));
        eventMap.put("Move arm to third", new ArmCommand(s_Arm, () -> activeConfig.place3));
        eventMap.put("Move arm to safe", new ArmCommand(s_Arm, Position.SAFE));
        eventMap.put("Eject cube", justPlaceCommand(s_Intake));
        eventMap.put("Chassis", new ArmCommand(s_Arm, Position.CHASSIS));
        eventMap.put("Ground intake",
            new ArmCommand(s_Arm, Position.INTAKE_GROUND)
                .alongWith(
                    Commands.runOnce(() -> { if (!s_Intake.isCube()) s_Intake.toggleCube(); })
                    .andThen(new WaitCommand(1.0))
                    .andThen(new IntakeCommand(s_Intake))));
        eventMap.put("Yoshi intake",
            new ArmCommand(s_Arm, Position.YOSHI)
                .alongWith(
                    Commands.runOnce(() -> { if (!s_Intake.isCube()) s_Intake.toggleCube(); })
                    .andThen(new WaitCommand(1.0))
                    .andThen(new IntakeCommand(s_Intake))));
        eventMap.put("Balance", new AutoBalanceCommand(s_Swerve, s_LED));

        autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(12, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(4.25, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Mirror Blue path to Red automatically
            s_Swerve);

        pieceChooser.addOption("--- Initial Piece ---", null);
        pieceChooser.setDefaultOption("Cube", Boolean.TRUE);
        pieceChooser.addOption("Cone", Boolean.FALSE);
        SmartDashboard.putData("Starting game piece", pieceChooser);

        placeChooser.addOption("--- First Position ---", null);
        placeChooser.addOption("High", Position.GRID_HIGH);
        placeChooser.addOption("Middle", Position.GRID_MID);
        placeChooser.addOption("Low", Position.GRID_LOW);
        placeChooser.addOption("Chassis", Position.CHASSIS);
        placeChooser.setDefaultOption("No movement", Position.NONE);
        SmartDashboard.putData("First placement location", placeChooser);

        placeChooser2.addOption("--- Second Position ---", null);
        placeChooser2.addOption("High", Position.GRID_HIGH);
        placeChooser2.addOption("Middle", Position.GRID_MID);
        placeChooser2.addOption("Low", Position.GRID_LOW);
        placeChooser2.setDefaultOption("Chassis", Position.CHASSIS);
        SmartDashboard.putData("Second placement location", placeChooser2);
        
        placeChooser3.addOption("--- Third Position ---", null);
        placeChooser3.addOption("High", Position.GRID_HIGH);
        placeChooser3.addOption("Middle", Position.GRID_MID);
        placeChooser3.addOption("Low", Position.GRID_LOW);
        placeChooser3.setDefaultOption("Chassis", Position.CHASSIS);
        SmartDashboard.putData("Third placement location", placeChooser3);

        pathChooser.addOption("--- Auto Routine ---", null);
        pathChooser.setDefaultOption("No auto (face intake away)", null);
        for (String pathName : getPathnames()) {
            pathChooser.addOption(pathName, pathName);
        }
        SmartDashboard.putData("Autonomous routine", pathChooser);

        quickChooser.addOption("--- Auto Quick Pick ---", null);
        quickChooser.setDefaultOption("Manual selection", null);
        for (String name : quickpicks.keySet()) {
          quickChooser.addOption(name, buildCommand(quickpicks.get(name)));
        }
        SmartDashboard.putData("Quick Picks", quickChooser);
    }

    public Command getAutoCommand() {
        Command autoCommand = quickChooser.getSelected();

        if (autoCommand == null) {
            AutoConfig config = new AutoConfig(pathChooser.getSelected(), pieceChooser.getSelected().booleanValue(),
                                               placeChooser.getSelected(), placeChooser2.getSelected(), placeChooser3.getSelected());
            autoCommand = buildCommand(config);
        }
        return autoCommand;
    }

    public Command buildCommand(AutoConfig config) {
        if (config == null || config.path == null)
            return null;

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(config.path, PathPlanner.getConstraintsFromPath(config.path));
        return Commands.runOnce(() -> {
            DriverStation.reportWarning("Running auto command built from path: " + config.path, false);
            activeConfig = config;
        }).andThen(autoBuilder.fullAuto(pathGroup));
    }

    private static List<String> getPathnames() {
        return Stream.of(new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles())
                .filter(file -> !file.isDirectory())
                .filter(file -> file.getName().matches(".*\\.path"))
                .map(File::getName)
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .sorted()
                .collect(Collectors.toList());
    }
}