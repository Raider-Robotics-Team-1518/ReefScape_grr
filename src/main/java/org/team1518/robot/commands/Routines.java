package org.team1518.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Robot;
import org.team1518.robot.subsystems.Swerve;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class Routines {

    private final Swerve swerve;

    public Routines(Robot robot) {
        swerve = robot.swerve;
    }

    /**
     * An example routine.
     */
    public Command example() {
        return sequence(print("Hello!"), swerve.stop(true).withTimeout(1.0)).withName("Routines.example()");
    }

    public Command driveOutOnly1() {
        return sequence(
            print("Drive Out Only 1"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.driveOutOnly1()")
        );
    }

    public Command driveOutCoralLevel1() {
        return sequence(
            print("Drive Out Coral Level 1"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.driveOutCoralLevel1()")
        );
    }

    public Command driveOutCoralLevel4() {
        return sequence(
            print("Drive Out Coral Level 4"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.driveOutCoralLevel4()")
        );
    }

    public Command driveOutAlgaeIntakeLevel2() {
        return sequence(
            print("Drive Out Algae Intake Level 2"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.driveOutAlgaeIntakeLevel2()")
        );
    }

    public Command coralLevel4AlgaeInBarge() {
        return sequence(
            print("Coral Level 4 Algae In Barge"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.coralLevel4AlgaeInBarge()")
        );
    }
    
    public Command driveOutAndRotate() {
        return sequence(
            print("Drive Out and Rotate"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.driveOutAndRotate()")
        );
    }
}
