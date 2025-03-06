// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to handle raising the lift to a specified level of the reef
 */

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

public class RaiseLift extends Command {

    private int level = 0;
    private double currentHeight = 0;
    private double targetHeight = 0;
    private boolean isDone = false;

    public RaiseLift(int level) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.elevatorSubsystem);
        // level will be one of 1, 2, 3, or 4
        this.level = level;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetHeight = Constants.Reef.coralLevels[this.level];
        isDone = false;
        // y
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentHeight = Robot.elevatorSubsystem.getCurrentHeight();
        if (Math.abs(targetHeight - currentHeight) > Constants.Tolerances.reefCoralHeightTolerance) {
            double v_sign = Math.signum(targetHeight - currentHeight);
            Robot.elevatorSubsystem.driveElevator(v_sign * (Constants.MotorSpeeds.elevatorPower));
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
