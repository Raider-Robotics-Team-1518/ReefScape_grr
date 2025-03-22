package org.team1518.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team1518.lib.util.GRRDashboard;
import org.team1518.lib.util.Profiler;
import org.team1518.lib.util.Tunable;
import org.team1518.robot.commands.Autos;
import org.team1518.robot.commands.ManualClimb;
import org.team1518.robot.commands.Routines;
import org.team1518.robot.commands.gamepiecemanipulator.IntakeAlgaeReef;
import org.team1518.robot.commands.gamepiecemanipulator.IntakeCoral;
import org.team1518.robot.commands.gamepiecemanipulator.ManualAlgaeIntake;
import org.team1518.robot.commands.gamepiecemanipulator.ManualCoralIntake;
import org.team1518.robot.commands.gamepiecemanipulator.ManualLift;
import org.team1518.robot.commands.gamepiecemanipulator.ManualWrist;
import org.team1518.robot.commands.gamepiecemanipulator.MoveToBarge;
import org.team1518.robot.commands.gamepiecemanipulator.MoveToEjectCoralAngle;
import org.team1518.robot.commands.gamepiecemanipulator.MoveToProcessor;
import org.team1518.robot.commands.gamepiecemanipulator.RaiseLift;
import org.team1518.robot.commands.gamepiecemanipulator.SetAlgaeTravelPosition;
import org.team1518.robot.commands.gamepiecemanipulator.SetIntakeCoralTravelPosition;
import org.team1518.robot.subsystems.Blinkies;
import org.team1518.robot.subsystems.ClimbSubsystem;
import org.team1518.robot.subsystems.ElevatorSubsystem;
import org.team1518.robot.subsystems.GamePieceManipulator;
import org.team1518.robot.subsystems.Swerve;
import org.team1518.robot.subsystems.WristSubsystem;

@Logged
public final class Robot extends TimedRobot {

    public final Swerve swerve;
    public static boolean invertedIMU = false; // Default is true for teleop
    public static WristSubsystem wristSubsystem;
    public static ElevatorSubsystem elevatorSubsystem;
    public static GamePieceManipulator gamePieceManipulator;
    public static ClimbSubsystem climbSubsystem;
    public static Blinkies m_blinkies;
    public static LimeLight limeLight;
    //private Boolean orientation = true;

    public final Routines routines;
    public final Autos autos;

    private final Joystick driver;
    private final CommandXboxController coDriver;
    private final CommandGenericHID buttonBox;
    private final JoystickButton swerveTare;
    private final JoystickButton trigger;

    //private final JoystickButton swerveMode;

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

        elevatorSubsystem = new ElevatorSubsystem();
        wristSubsystem = new WristSubsystem();
        gamePieceManipulator = new GamePieceManipulator();
        climbSubsystem = new ClimbSubsystem();
        CameraServer.startAutomaticCapture(); // enable for USB camera

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize controllers
        driver = new Joystick(Constants.kDriver);
        coDriver = new CommandXboxController(Constants.kCoDriver);
        buttonBox = new CommandGenericHID(Constants.kButtonBox);
        swerveTare = new JoystickButton(driver, 15);
        trigger = new JoystickButton(driver, 1);
        //swerveMode = new JoystickButton(driver, 11);

        // Set default commands
        swerve.setDefaultCommand(swerve.drive(driver::getX, driver::getY, () -> Math.pow(driver.getZ(), 3) * 0.9));

        // Create triggers
        RobotModeTriggers.autonomous().whileTrue(GRRDashboard.runSelectedAuto());

        // Driver bindings
        swerveTare.onTrue(swerve.tareRotation());
        trigger.whileTrue(swerve.driveToReef(driver::getX, driver::getY, () -> Math.pow(driver.getZ(), 3) * 0.9));
        //swerveMode.toggleOnTrue(switchSwerveMode());

        // Co-driver bindings for manual operations
        coDriver.y().whileTrue(new ManualWrist(0.25)); // .onFalse(new ManualWrist(0));
        coDriver.a().whileTrue(new ManualWrist(-0.25)); // .onFalse(new ManualWrist(0));
        coDriver.povUp().whileTrue(new ManualLift(Constants.MotorSpeeds.elevatorPower));
        coDriver.povDown().whileTrue(new ManualLift(Constants.MotorSpeeds.elevatorPowerDn));
        coDriver.rightTrigger().whileTrue(new ManualAlgaeIntake(Constants.MotorSpeeds.algaeManualEjectMotorSpeed));
        coDriver.leftTrigger().whileTrue(new ManualAlgaeIntake(Constants.MotorSpeeds.algaeManualIntakeMotorSpeed));
        coDriver.leftBumper().whileTrue(new ManualCoralIntake(Constants.MotorSpeeds.coralManualIntakeMotorSpeed));
        coDriver.rightBumper().whileTrue(new ManualCoralIntake(Constants.MotorSpeeds.slowAlgaeManualIntakeMotorSpeed));

        //Clime bindings
        coDriver.x().whileTrue(new ManualClimb(Constants.MotorSpeeds.climbMotorSpeed)).onFalse(new ManualClimb(0));
        coDriver.b().whileTrue(new ManualClimb(-Constants.MotorSpeeds.climbMotorSpeed)).onFalse(new ManualClimb(0));

        // Algae Intake Options
        buttonBox
            .button(11)
            .onTrue(
                new SetAlgaeTravelPosition()
                    .andThen(
                        new IntakeAlgaeReef(1).andThen(
                            Commands.race(new ManualAlgaeIntake(-.4), Commands.waitSeconds(0.25)) //
                        )
                    )
                    .andThen(new SetAlgaeTravelPosition())
            );
        buttonBox
            .button(5)
            .onTrue(
                new SetAlgaeTravelPosition()
                    .andThen(
                        new IntakeAlgaeReef(2).andThen(
                            Commands.race(new ManualAlgaeIntake(-.4), Commands.waitSeconds(0.25)) //
                        )
                    )
                    .andThen(new SetAlgaeTravelPosition())
            );
        buttonBox
            .button(6)
            .onTrue(
                new SetAlgaeTravelPosition()
                    .andThen(
                        new IntakeAlgaeReef(3).andThen(
                            Commands.race(new ManualAlgaeIntake(-.4), Commands.waitSeconds(0.25)) //
                        )
                    )
                //.andThen(new SetAlgaeTravelPosition())
            );
        buttonBox
            .button(7)
            .onTrue(
                new SetAlgaeTravelPosition()
                    .andThen(
                        new IntakeAlgaeReef(4).andThen(
                            Commands.race(new ManualAlgaeIntake(-.4), Commands.waitSeconds(0.25)) //
                        )
                    )
                //.andThen(new SetAlgaeTravelPosition())
            );
        // Algae Deposit on Barge
        buttonBox.button(9).onTrue(new SetAlgaeTravelPosition().andThen(new MoveToBarge()));
        buttonBox.button(8).onTrue(new SetAlgaeTravelPosition().andThen((new MoveToProcessor())));

        // Intake Coral from Feeder
        buttonBox.button(10).onTrue(new IntakeCoral().andThen(new SetIntakeCoralTravelPosition()));
        // Coral Deposit on Reef Options
        buttonBox
            .button(4)
            .onTrue(new SetIntakeCoralTravelPosition().andThen(new RaiseLift(1)).andThen(new MoveToEjectCoralAngle(1)));
        buttonBox
            .button(3)
            .onTrue(new SetIntakeCoralTravelPosition().andThen(new RaiseLift(2)).andThen(new MoveToEjectCoralAngle(2)));
        buttonBox
            .button(2)
            .onTrue(new SetIntakeCoralTravelPosition().andThen(new RaiseLift(3)).andThen(new MoveToEjectCoralAngle(3)));
        buttonBox
            .button(1)
            .onTrue(new SetIntakeCoralTravelPosition().andThen(new RaiseLift(4)).andThen(new MoveToEjectCoralAngle(4)));
    }

    /*private Command switchSwerveMode() {
        if (this.orientation) {
            swerve.setDefaultCommand(
                swerve.driveRobotOriented(driver::getX, driver::getY, () -> Math.pow(driver.getZ(), 3) * 0.9)
            );
            this.orientation = false;
        } else {
            this.orientation = true;
            swerve.setDefaultCommand(swerve.drive(driver::getX, driver::getY, () -> Math.pow(driver.getZ(), 3) * 0.9));
        }
        return null;
    }*/

    @Override
    public void teleopInit() {
        // Before measurements can be read from the sensor, this must be called
        // to start a background thread that will periodically poll all
        // enabled sensors and store their measured range.
        invertedIMU = true;
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
        SmartDashboard.putNumber("Horizontal Offset", limeLight.getTargetOffsetHorizontal());
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void autonomousInit() {
        Command auto = autos.getAuto();
        auto.schedule();
    }
}
