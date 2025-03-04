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

public class RaiseToCoralIntake extends Command {

    private double currentHeight = 0;
    private double targetHeight = Constants.Reef.targetCoralIntakeHeight;
    private boolean isDone = false;
    private boolean isCoralLoaded = false;

    public RaiseToCoralIntake() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    /* @Override
    public void execute() {
        currentHeight = Robot.elevatorSubsystem.getCurrentHeight();
        if (Math.abs(targetHeight - currentHeight) > Constants.Tolerances.reefCoralHeightTolerance) {
            double v_sign = Math.signum(targetHeight - currentHeight);
            Robot.elevatorSubsystem.driveElevator(v_sign * (Constants.MotorSpeeds.elevatorPower));
        } else {
            Robot.elevatorSubsystem.stopElevator();
            isDone = true;
        }
    } */

    @Override
    public void execute() {
        /*if (targetHeight == 5.9 && isCoralLoaded) {
            isDone = true;
        } else {
            // set height to correct
            currentHeight = Robot.elevatorSubsystem.getCurrentHeight();
            // Move arm up or down to target arm angle
            if (Math.abs(targetHeight - currentHeight) > Constants.Tolerances.reefCoralHeightTolerance) {
                double v_sign = Math.signum(targetHeight - currentHeight);
                Robot.elevatorSubsystem.driveElevator(v_sign * (Constants.MotorSpeeds.elevatorPower));
            } else {
                Robot.elevatorSubsystem.stopElevator();
                isDone = true;
            }*/
        /*if (!Robot.gamePieceManipulator.isCoralLoaded()) {
                Robot.gamePieceManipulator.intakeCoral();
            } else {
                Robot.gamePieceManipulator.stopGamePieceMotor();
                isCoralLoaded = true;
            }
        }*/
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
