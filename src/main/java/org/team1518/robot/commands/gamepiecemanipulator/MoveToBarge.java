// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to eject the algae into the processor
 */

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToBarge extends Command {

    // private Timer timer;
    private boolean isDone = false;
    private boolean isAlgaeLoaded = false;
    private boolean isAtHeight = false;
    private double current_angle = 0;
    private double current_height = 0;

    public MoveToBarge() {
        addRequirements(Robot.gamePieceManipulator, Robot.wristSubsystem, Robot.elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isDone = false;
        isAlgaeLoaded = false;
        isAtHeight = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        isAlgaeLoaded = Robot.gamePieceManipulator.isAlgaeLoaded();
        if (isAlgaeLoaded) {
            if (!isAtHeight) {
                // set height to correct
                current_height = Robot.elevatorSubsystem.getCurrentHeight();
                // Move elevator up or down to target height
                if (
                    Math.abs(Constants.Limits.elevatorMax - current_height) >
                    Constants.Tolerances.reefCoralHeightTolerance
                ) {
                    double v_sign = Math.signum(Constants.Limits.elevatorMax - current_height);
                    Robot.elevatorSubsystem.driveElevator(v_sign * (Constants.MotorSpeeds.elevatorPower));
                } else {
                    Robot.elevatorSubsystem.stopElevator();
                    isAtHeight = true;
                }
            } else {
                // set arm to correct angle
                current_angle = Robot.wristSubsystem.getWristPosition();
                // Calculate power curve proportional
                double armRotationPower = Math.abs(Constants.Reef.targetAlgaeEjectBargeAngle - current_angle) / 300;
                // Move arm up or down to target arm angle
                if (
                    Math.abs(Constants.Reef.targetAlgaeEjectBargeAngle - current_angle) >
                    Constants.Tolerances.coralIntakeAngleTolerance
                ) {
                    double v_sign = Math.signum(Constants.Reef.targetAlgaeEjectBargeAngle - current_angle);
                    Robot.wristSubsystem.setWristSpeed(v_sign * (armRotationPower));
                } else {
                    isDone = true;
                }
            }
        } else {
            isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // timer.stop();
        Robot.wristSubsystem.stopWrist();
        Robot.elevatorSubsystem.stopElevator();
        isDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
