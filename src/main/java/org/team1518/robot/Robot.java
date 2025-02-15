package org.team1518.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team1518.lib.util.GRRDashboard;
import org.team1518.lib.util.Profiler;
import org.team1518.lib.util.Tunable;
import org.team1518.robot.commands.Autos;
import org.team1518.robot.commands.Routines;
import org.team1518.robot.commands.gamepiecemanipulator.ManualLift;
import org.team1518.robot.subsystems.Blinkies;
import org.team1518.robot.subsystems.ElevatorSubsystem;
import org.team1518.robot.subsystems.GamePieceManipulator;
import org.team1518.robot.subsystems.Swerve;
import org.team1518.robot.subsystems.WristSubsystem;

@Logged
public final class Robot extends TimedRobot {

    public final Swerve swerve;
    public static WristSubsystem wristSubsystem;
    public static ElevatorSubsystem elevatorSubsystem;
    public static GamePieceManipulator gamePieceManipulator;
    public static Blinkies m_blinkies;
    public static LimeLight limeLight;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        // Epilogue.getConfig().root = "/Telemetry";

        // Initialize subsystems
        swerve = new Swerve();
        /* LED Lights */
        m_blinkies = new Blinkies();
        /* Lime Light Cameras */
        limeLight = new LimeLight();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        elevatorSubsystem = new ElevatorSubsystem();
        wristSubsystem = new WristSubsystem();
        gamePieceManipulator = new GamePieceManipulator();
        CameraServer.startAutomaticCapture();

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);
        coDriver = new CommandXboxController(Constants.kCoDriver);

        // Set default commands
        swerve.setDefaultCommand(swerve.drive(driver::getLeftX, driver::getLeftY, () -> 0));

        // Create triggers
        RobotModeTriggers.autonomous().whileTrue(GRRDashboard.runSelectedAuto());

        // Driver bindings
        driver.povLeft().onTrue(swerve.tareRotation());

        // Co-driver bindings
        coDriver.a().onTrue(none());
        coDriver.y().whileTrue(new ManualLift(0.45));
        coDriver.b().whileTrue(new ManualLift(-0.25));
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("RobotPeriodic");
        Profiler.run("CommandScheduler", () -> CommandScheduler.getInstance().run());
        // Profiler.run("Epilogue", () -> Epilogue.update(this));
        Profiler.run("GRRDashboard", GRRDashboard::update);
        Profiler.run("Tunables", Tunable::update);
        Profiler.end();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
