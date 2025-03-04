package org.team1518.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team1518.lib.util.GRRDashboard;
import org.team1518.lib.util.Profiler;
import org.team1518.lib.util.Tunable;
import org.team1518.robot.commands.Autos;
import org.team1518.robot.commands.Routines;
import org.team1518.robot.commands.gamepiecemanipulator.IntakeAlgaeReef;
import org.team1518.robot.commands.gamepiecemanipulator.IntakeCoral;
import org.team1518.robot.commands.gamepiecemanipulator.ManualAlgaeIntake;
import org.team1518.robot.commands.gamepiecemanipulator.ManualCoralIntake;
import org.team1518.robot.commands.gamepiecemanipulator.ManualLift;
import org.team1518.robot.commands.gamepiecemanipulator.ManualWrist;
import org.team1518.robot.commands.gamepiecemanipulator.MoveToBarge;
import org.team1518.robot.commands.gamepiecemanipulator.MoveToEjectCoralAngle;
import org.team1518.robot.commands.gamepiecemanipulator.RaiseLift;
import org.team1518.robot.commands.gamepiecemanipulator.SetTravelPosition;
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

    public static Rev2mDistanceSensor distanceSensor;

    public final Routines routines;
    public final Autos autos;

    private final Joystick driver;
    private final CommandXboxController coDriver;
    private final CommandGenericHID buttonBox;

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

        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

        elevatorSubsystem = new ElevatorSubsystem();
        wristSubsystem = new WristSubsystem();
        gamePieceManipulator = new GamePieceManipulator();
        CameraServer.startAutomaticCapture(); // enable for USB camera

        // Initialize controllers
        driver = new Joystick(Constants.kDriver);
        coDriver = new CommandXboxController(Constants.kCoDriver);
        buttonBox = new CommandGenericHID(Constants.kButtonBox);

        // Set default commands
        swerve.setDefaultCommand(swerve.drive(driver::getX, driver::getY, () -> Math.pow(driver.getZ(), 3) * 0.75));

        // Create triggers
        RobotModeTriggers.autonomous().whileTrue(GRRDashboard.runSelectedAuto());

        // Driver bindings
        // driver.povLeft().onTrue(swerve.tareRotation());

        // Co-driver bindings
        coDriver.y().whileTrue(new ManualWrist(0.25)); // .onFalse(new ManualWrist(0));
        coDriver.a().whileTrue(new ManualWrist(-0.25)); // .onFalse(new ManualWrist(0));
        coDriver.povUp().whileTrue(new ManualLift(Constants.MotorSpeeds.elevatorPower));
        coDriver.povDown().whileTrue(new ManualLift(Constants.MotorSpeeds.elevatorPowerDn));
        coDriver.rightTrigger().whileTrue(new ManualAlgaeIntake(Constants.MotorSpeeds.algaeManualEjectMotorSpeed)); // Eject coral and algae
        coDriver.leftTrigger().whileTrue(new ManualAlgaeIntake(Constants.MotorSpeeds.algaeManualIntakeMotorSpeed)); // .onFalse(new
        // ManualAlgaeIntake(0));
        coDriver.leftBumper().whileTrue(new ManualCoralIntake(Constants.MotorSpeeds.coralManualIntakeMotorSpeed)); // .onFalse(new
        // ManualCoralIntake(0));
        buttonBox.button(10).onTrue(new IntakeCoral().andThen(new SetTravelPosition()));
        buttonBox.button(9).onTrue(new SetTravelPosition().andThen(new MoveToBarge()));
        buttonBox.button(6).onTrue(new IntakeAlgaeReef(2)/* .andThen(new SetTravelPosition()) */);
        buttonBox.button(7).onTrue(new IntakeAlgaeReef(3)/* .andThen(new SetTravelPosition()) */);
        buttonBox
            .button(4)
            .onTrue(new SetTravelPosition().andThen(new RaiseLift(1)).andThen(new MoveToEjectCoralAngle(1)));
        buttonBox
            .button(3)
            .onTrue(new SetTravelPosition().andThen(new RaiseLift(2)).andThen(new MoveToEjectCoralAngle(2)));
        buttonBox
            .button(2)
            .onTrue(new SetTravelPosition().andThen(new RaiseLift(3)).andThen(new MoveToEjectCoralAngle(3)));
        buttonBox
            .button(1)
            .onTrue(new SetTravelPosition().andThen(new RaiseLift(4)).andThen(new MoveToEjectCoralAngle(4)));
    }

    @Override
    public void teleopInit() {
        // Before measurements can be read from the sensor, this must be called
        // to start a background thread that will periodically poll all
        // enabled sensors and store their measured range.
        distanceSensor.setAutomaticMode(true);
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
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Distance", limeLight.getDistanceToTarget());
        if (Robot.distanceSensor.isRangeValid()) {
            SmartDashboard.putNumber("Raw distance", Robot.distanceSensor.getRange());
        }
    }

    @Override
    public void testPeriodic() {}
}
