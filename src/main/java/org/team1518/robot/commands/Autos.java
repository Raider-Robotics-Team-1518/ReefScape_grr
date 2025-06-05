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
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;
import org.team1518.robot.commands.gamepiecemanipulator.IntakeAlgaeReef;
import org.team1518.robot.commands.gamepiecemanipulator.ManualAlgaeIntake;
import org.team1518.robot.commands.gamepiecemanipulator.ManualCoralIntake;
import org.team1518.robot.commands.gamepiecemanipulator.MoveToBarge;
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
        autoChooser.addOption("Drive Out Only", driveOutOnly1());
        autoChooser.addOption("Score Level 1", driveOutCoralLevel1());
        autoChooser.addOption("Drive out Algae 2", driveOutIntakeAlgae());
        autoChooser.addOption("Score Level 4", driveOutCoralLevel4());
        autoChooser.addOption("Algae In Barge", driveOutIntakeAlgaeScoreBarge());
        autoChooser.addOption("Drive Out and Rotate", driveOutAndRotate());
        autoChooser.addOption("Coral Level 4, Algae In Barge", coralLevel4AlgaeInBarge());
        autoChooser.addOption("Drive Out and Rotate", scoreCoralAndAlgae());
        SmartDashboard.putData("Select Auto", autoChooser);
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
        AutoRoutine routine = factory.newRoutine("driveOutOnly1");
        AutoTrajectory driveOutOnly1 = routine.trajectory("Drive_Out_Only_1_(Level_1_Coral)");

        routine.active().onTrue(sequence(driveOutOnly1.resetOdometry(), driveOutOnly1.cmd()));

        driveOutOnly1.done().onTrue(sequence(routines.driveOutOnly1(), swerve.finishAuto()));

        return routine.cmd();
    }

    private Command driveOutCoralLevel4() {
        AutoRoutine routine = factory.newRoutine("driveOutCoralLevel4");
        AutoTrajectory driveOutOnly2 = routine.trajectory("Drive_Out_Only_2_(Level_234_Coral)");

        routine
            .active()
            .onTrue(
                sequence(
                    driveOutOnly2.resetOdometry(),
                    new SetIntakeCoralTravelPosition().andThen(new RaiseLift(4)),
                    driveOutOnly2.cmd()
                )
            );

        driveOutOnly2
            .done()
            .onTrue(sequence(routines.driveOutCoralLevel4(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToEjectCoralAngle(4),
                    new ManualCoralIntake(Constants.MotorSpeeds.coralAutoEjectMotorSpeedL234)
                )
            );

        return routine.cmd();
    }

    private Command driveOutCoralLevel1() {
        AutoRoutine routine = factory.newRoutine("driveOutCoralLevel1");
        AutoTrajectory driveOutOnly1 = routine.trajectory("Drive_Out_Only_1_(Level_1_Coral)");

        routine
            .active()
            .onTrue(
                sequence(
                    driveOutOnly1.resetOdometry(),
                    new SetIntakeCoralTravelPosition().andThen(new RaiseLift(1)),
                    driveOutOnly1.cmd()
                )
            );

        driveOutOnly1
            .done()
            .onTrue(sequence(routines.driveOutCoralLevel1(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToEjectCoralAngle(1),
                    new ManualAlgaeIntake(Constants.MotorSpeeds.coralAutoEjectMotorSpeedL1)
                )
            );

        return routine.cmd();
    }

    private Command driveOutIntakeAlgae() {
        AutoRoutine routine = factory.newRoutine("driveOutAlgaeIntakeLevel2");
        AutoTrajectory driveOutOnly3 = routine.trajectory("Drive_Out_Only_3_(Algae)");

        routine
            .active()
            .onTrue(
                sequence(
                    driveOutOnly3.resetOdometry(),
                    new SetAlgaeTravelPosition()
                        .andThen(
                            Commands.parallel(
                                // ground is level 1, so 3 is level 2 on the reef
                                Commands.race(new IntakeAlgaeReef(3), Commands.waitSeconds(5)),
                                driveOutOnly3.cmd()
                            )
                        )
                )
            );

        driveOutOnly3.done().onTrue(sequence(routines.driveOutAlgaeIntakeLevel2(), swerve.finishAuto()));

        return routine.cmd();
    }

    private Command driveOutIntakeAlgaeScoreBarge() {
        AutoRoutine routine = factory.newRoutine("driveOutAlgaeIntakeLevel2");
        AutoTrajectory driveOutOnly3 = routine.trajectory("Drive_Out_Only_3_(Algae)");
        AutoTrajectory reefToBarge = routine.trajectory("Reef_To_Barge");

        routine
            .active()
            .onTrue(
                sequence(
                    driveOutOnly3.resetOdometry(),
                    new SetAlgaeTravelPosition()
                        .andThen(
                            Commands.parallel(
                                Commands.race(new IntakeAlgaeReef(3), Commands.waitSeconds(3)),
                                driveOutOnly3.cmd()
                            )
                        )
                )
            );

        driveOutOnly3
            .done()
            .onTrue(sequence(routines.driveOutAlgaeIntakeLevel2(), swerve.finishAuto()))
            .onTrue(Commands.sequence(new SetAlgaeTravelPosition(), reefToBarge.cmd()));

        reefToBarge
            .done()
            .onTrue(sequence(routines.driveOutAlgaeIntakeLevel2(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToBarge(),
                    new ManualCoralIntake(Constants.MotorSpeeds.coralManualIntakeMotorSpeed)
                )
            );

        return routine.cmd();
    }

    private Command coralLevel4AlgaeInBarge() {
        AutoRoutine routine = factory.newRoutine("coralLevel4AlgaeInBarge");
        AutoTrajectory driveOutOnly2 = routine.trajectory("Drive_Out_Only_2_(Level_234_Coral)");
        AutoTrajectory reefToBarge = routine.trajectory("Reef_To_Barge");
        AutoTrajectory reposition = routine.trajectory("Reposition");

        routine
            .active()
            .onTrue(
                sequence(
                    driveOutOnly2.resetOdometry(),
                    new SetIntakeCoralTravelPosition().andThen(new RaiseLift(4)),
                    driveOutOnly2.cmd()
                )
            );

        driveOutOnly2
            .done()
            .onTrue(sequence(routines.coralLevel4AlgaeInBarge(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToEjectCoralAngle(4),
                    new ManualCoralIntake(Constants.MotorSpeeds.coralAutoEjectMotorSpeedL234),
                    Commands.parallel(reposition.cmd(), new IntakeAlgaeReef(3))
                )
            );

        reposition
            .done()
            .onTrue(sequence(routines.coralLevel4AlgaeInBarge(), swerve.finishAuto()))
            .onTrue(Commands.sequence(new SetAlgaeTravelPosition(), reefToBarge.cmd()));

        reefToBarge
            .done()
            .onTrue(sequence(routines.coralLevel4AlgaeInBarge(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToBarge(),
                    new ManualCoralIntake(Constants.MotorSpeeds.coralManualIntakeMotorSpeed)
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

    private Command scoreCoralAndAlgae() {
        AutoRoutine routine = factory.newRoutine("scoreCoralAndAlgae");
        AutoTrajectory driveOutOnly2 = routine.trajectory("Drive_Out_Only_2_(Level_234_Coral)");
        AutoTrajectory reefToBarge = routine.trajectory("Reef_To_Barge");
        AutoTrajectory repositionP1 = routine.trajectory("RepositionP1");
        AutoTrajectory repositionP2 = routine.trajectory("RepositionP2");

        routine
            .active()
            .onTrue(
                sequence(
                    driveOutOnly2.resetOdometry(),
                    new SetIntakeCoralTravelPosition().andThen(new RaiseLift(1)),
                    driveOutOnly2.cmd()
                )
            );

        driveOutOnly2
            .done()
            .onTrue(sequence(routines.driveOutCoralLevel1(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToEjectCoralAngle(1),
                    new ManualAlgaeIntake(Constants.MotorSpeeds.coralAutoEjectMotorSpeedL1)
                )
            );

        repositionP1
            .done()
            .onTrue(sequence(routines.repositionP1(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new IntakeAlgaeReef(3),
                    new ManualAlgaeIntake(Constants.MotorSpeeds.coralAutoEjectMotorSpeedL1)
                )
            );

        repositionP2
            .done()
            .onTrue(sequence(routines.repositionP2(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    //new IntakeAlgaeReef(4),
                    new ManualAlgaeIntake(Constants.MotorSpeeds.coralAutoEjectMotorSpeedL1)
                )
            );

        reefToBarge
            .done()
            .onTrue(sequence(routines.reefToBarge(), swerve.finishAuto()))
            .onTrue(
                Commands.sequence(
                    new MoveToBarge(),
                    Commands.race(
                        new ManualCoralIntake(Constants.MotorSpeeds.algaeEjectMotorSpeed),
                        Commands.waitSeconds(2)
                    ),
                    new SetAlgaeTravelPosition(),
                    new IntakeAlgaeReef(0)
                )
            );

        return routine.cmd();
    }
}
/*
 * private SequentialCommandGroup moveToLevel4() {
 * return new SetIntakeCoralTravelPosition().andThen(new
 * RaiseLift(4)).andThen(new MoveToEjectCoralAngle(4));
 * }
 *
 * private SequentialCommandGroup autoA3() {
 * return new SetAlgaeTravelPosition()
 * .andThen(new IntakeAlgaeReef(3))
 * .andThen(Commands.race(new ManualAlgaeIntake(-.4),
 * Commands.waitSeconds(0.25))); //
 * }
 */
