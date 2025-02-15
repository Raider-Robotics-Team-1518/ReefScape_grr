// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to handle raising the lift to a specified level of the reef
 */

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Robot;

public class ManualLift extends Command {

    private double speed = 0;
    private double currentHeight = 0;
    private double targetHeight = 0;
    private boolean isDone = false;

    public ManualLift(double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.elevatorSubsystem);
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(speed) > 0) {
            Robot.elevatorSubsystem.driveElevator(speed);
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
