package org.team1518.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;
import org.team1518.robot.commands.gamepiecemanipulator.IntakeAlgaeReef;
import org.team1518.robot.commands.gamepiecemanipulator.ManualAlgaeIntake;
import org.team1518.robot.commands.gamepiecemanipulator.MoveToEjectCoralAngle;
import org.team1518.robot.commands.gamepiecemanipulator.RaiseLift;
import org.team1518.robot.commands.gamepiecemanipulator.SetAlgaeTravelPosition;
import org.team1518.robot.commands.gamepiecemanipulator.SetIntakeCoralTravelPosition;
import org.team1518.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private final Swerve swerve;
    private final Routines routines;

    private final AutoFactory factory;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public Autos(Robot robot) {
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto factory
        factory = new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);

        // Add autonomous modes to the dashboard
        autoChooser.addOption("Example", example());
        autoChooser.addOption("Drive Out 1", driveOutOnly1());
        autoChooser.addOption("Drive Out and Rotate", driveOutAndRotate());
        autoChooser.addOption("Drive to Reef 1", driveToReef1());
        SmartDashboard.putData("Select Auto", autoChooser);
        /*GRRDashboard.setTrajectoryCache(factory.cache());
        GRRDashboard.addAuto("Example", "example", example());
        GRRDashboard.addAuto("Drive Out 1", "Drive_Out_Only_1", driveOutOnly1());*/
        //SmartDashboard.putData("Select Auto", GRRDashboard);
        // GRRDashboard.addAuto("Drive Out 2", "Drive_Out_Only_2", driveOutOnly2());
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }

    private Command example() {
        AutoRoutine routine = factory.newRoutine("Example");
        AutoTrajectory exampleTraj = routine.trajectory("example");

        routine.active().onTrue(sequence(exampleTraj.resetOdometry(), exampleTraj.cmd()));
        exampleTraj.done().onTrue(sequence(routines.example(), swerve.finishAuto()));

        return routine.cmd();
    }

    private Command driveOutOnly1() {
        AutoRoutine routine = factory.newRoutine("driveOut1");
        AutoTrajectory driveOut1 = routine.trajectory("Drive_Out_Only_1");

        routine
            .active()
            .onTrue(
                sequence(
                    driveOut1.resetOdometry(),
                    new SetIntakeCoralTravelPosition().andThen(new RaiseLift(1)),
                    driveOut1.cmd()
                )
            );
        driveOut1
            .done()
            .onTrue(sequence(routines.driveOutOnly1(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToEjectCoralAngle(1),
                    new ManualAlgaeIntake(Constants.MotorSpeeds.coralAutoEjectMotorSpeed)
                )
            );

        return routine.cmd();
    }

    private Command driveOutAndRotate() {
        AutoRoutine routine = factory.newRoutine("driveOutAndRotate");
        AutoTrajectory driveOutAndRotate = routine.trajectory("Drive_Out_And_Rotate");

        routine.active().onTrue(sequence(driveOutAndRotate.resetOdometry(), driveOutAndRotate.cmd()));
        driveOutAndRotate.done().onTrue(sequence(routines.driveOutAndRotate(), swerve.finishAuto()));

        return routine.cmd();
    }

    private Command driveToReef1() {
        AutoRoutine routine = factory.newRoutine("driveToReef1");
        AutoTrajectory driveToReef1 = routine.trajectory("Drive_To_Reef_(pos1)");

        routine.active().onTrue(sequence(driveToReef1.resetOdometry(), driveToReef1.cmd()));
        driveToReef1.done().onTrue(sequence(routines.driveToReef1(), swerve.finishAuto()));

        return routine.cmd();
    }

    private SequentialCommandGroup moveToLevel4() {
        return new SetIntakeCoralTravelPosition().andThen(new RaiseLift(4)).andThen(new MoveToEjectCoralAngle(4));
    }

    private SequentialCommandGroup autoA3() {
        return new SetAlgaeTravelPosition()
            .andThen(new IntakeAlgaeReef(3))
            .andThen(Commands.race(new ManualAlgaeIntake(-.4), Commands.waitSeconds(0.25))); //
    }
}
