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
import org.team1518.robot.Robot;
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

        routine.active().onTrue(sequence(driveOut1.resetOdometry(), driveOut1.cmd()));
        driveOut1.done().onTrue(sequence(routines.driveOutOnly1(), swerve.finishAuto()));

        return routine.cmd();
    }
}
