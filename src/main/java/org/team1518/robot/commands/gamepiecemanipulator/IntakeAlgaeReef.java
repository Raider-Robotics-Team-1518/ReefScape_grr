// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands.gamepiecemanipulator;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

public class IntakeAlgaeReef extends Command {

    // private Timer timer;
    private int level = 0;
    private boolean isDone = false;
    private boolean isAtAngle = false;
    private boolean isAlgaeLoaded = false;
    private boolean isAtHeight = false;
    private double current_angle = 0;
    private double targetAngle = 0;
    private double current_height = 0;
    private double targetHeight = 0;

    public IntakeAlgaeReef(int level) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.wristSubsystem, Robot.elevatorSubsystem, Robot.gamePieceManipulator);
        //this.targetAlgaeIntakeAngle = targetAlgaeIntakeAngle;
        // level will be one of LOW, 2, 3
        this.level = level;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // timer = new Timer();
        isDone = false;
        isAtAngle = false;
        isAlgaeLoaded = false;
        isAtHeight = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isAtAngle && isAtHeight && Robot.gamePieceManipulator.isAlgaeLoaded()) {
            isDone = true;
        } else {
            // set arm to correct angle
            targetAngle = Constants.Reef.algaeAngles[this.level];
            current_angle = Robot.wristSubsystem.getWristPosition();
            // Calculate power curve proportional
            double armRotationPower = Math.abs(this.targetAngle - current_angle) / 300 + 0.15;
            // Move arm up or down to target arm angle
            if (Math.abs(this.targetAngle - current_angle) > Constants.Tolerances.algaeIntakeAngleTolerance) {
                double v_sign = Math.signum(this.targetAngle - current_angle);
                Robot.wristSubsystem.setWristSpeed(v_sign * (armRotationPower));
            } else {
                Robot.wristSubsystem.stopWrist();
                isAtAngle = true;
            }
            // set height to correct
            current_height = Robot.elevatorSubsystem.getCurrentHeight();
            targetHeight = Constants.Reef.algaeLevels[this.level];
            // Move elevator up or down to target height
            if (Math.abs(targetHeight - current_height) > Constants.Tolerances.reefAlgaeHeightTolerance) {
                double v_sign = Math.signum(targetHeight - current_height);
                Robot.elevatorSubsystem.driveElevator(v_sign * (Constants.MotorSpeeds.elevatorPower));
            } else {
                Robot.elevatorSubsystem.stopElevator();
                isAtHeight = true;
            }
            if (!Robot.gamePieceManipulator.isAlgaeLoaded()) {
                Robot.gamePieceManipulator.intakeAlgae();
            } else {
                Robot.gamePieceManipulator.stopGamePieceMotor();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // timer.stop();
        Robot.wristSubsystem.stopWrist();
        Robot.gamePieceManipulator.stopGamePieceMotor();
        Robot.elevatorSubsystem.stopElevator();
        isDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
