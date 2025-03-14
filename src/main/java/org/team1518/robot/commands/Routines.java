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

    public Command driveOutAndRotate() {
        return sequence(
            print("Drive Out and Rotate"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.driveOutAndRotate()")
        );
    }

    public Command driveToReef1() {
        return sequence(
            print("Drive to Reef 1"),
            swerve.stop(true).withTimeout(1.0).withName("Routines.driveToReef1()")
        );
    }
}
