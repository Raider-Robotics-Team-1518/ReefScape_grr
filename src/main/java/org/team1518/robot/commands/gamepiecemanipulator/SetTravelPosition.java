// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTravelPosition extends Command {

    private boolean isDone = false;
    private boolean isAtHeight = false;
    private boolean isAtAngle = false;

    /** Creates a new SetTravelPosition. */
    public SetTravelPosition() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.elevatorSubsystem, Robot.wristSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isAtHeight && isAtAngle) {
            isDone = true;
        } else {
            //Move the elevator to 2.5"
            if (Robot.elevatorSubsystem.getCurrentHeight() < Constants.Reef.coralIntakeHeight) {
                Robot.elevatorSubsystem.driveElevator(Constants.MotorSpeeds.elevatorPower); //move the carriage up
            } else if (Robot.elevatorSubsystem.getCurrentHeight() > Constants.Reef.coralIntakeHeight) {
                Robot.elevatorSubsystem.driveElevator(-Constants.MotorSpeeds.elevatorPower); //move the carriage down
            } else {
                Robot.elevatorSubsystem.stopElevator();
                isAtHeight = true;
            }

            //Move the wrist to the intake position
            if (Robot.wristSubsystem.getWristPosition() < Constants.Reef.targetCoralIntakeAngle) {
                Robot.wristSubsystem.setWristSpeed(Constants.MotorSpeeds.wristPower); //Move the wrist up
            } else if (Robot.wristSubsystem.getWristPosition() > Constants.Reef.targetCoralIntakeAngle) {
                Robot.wristSubsystem.setWristSpeed(-Constants.MotorSpeeds.wristPower);
            } else {
                Robot.wristSubsystem.stopWrist();
                isAtAngle = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
