// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Lower the elevator to its lowest position.
 */

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowerLift extends Command {

    private double currentHeight = 0;
    private double targetHeight = 0;
    private boolean isDone = false;

    public LowerLift() {
        addRequirements(Robot.elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentHeight = Robot.elevatorSubsystem.getCurrentHeight();
        if (currentHeight > targetHeight + Constants.Tolerances.minElevatorHeightTolerance) {
            Robot.elevatorSubsystem.driveElevator(-Constants.MotorSpeeds.elevatorPower);
        } else {
            Robot.elevatorSubsystem.stopElevator();
            isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevatorSubsystem.stopElevator();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
