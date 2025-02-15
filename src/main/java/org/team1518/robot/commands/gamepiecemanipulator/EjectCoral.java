// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to eject the coral onto the reef
 */

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

public class EjectCoral extends Command {

    private Timer timer;
    private boolean isDone = false;
    private double current_angle = Robot.wristSubsystem.getWristPosition();
    private double targetArmAngle = Constants.Reef.coralEjectAngleLevel23;

    public EjectCoral(int level) {
        addRequirements(Robot.wristSubsystem, Robot.wristSubsystem);
        if (level == 1) {
            this.targetArmAngle = Constants.Reef.coralEjectAngleLevel1;
        } else if (level == 2 || level == 3) {
            this.targetArmAngle = Constants.Reef.coralEjectAngleLevel23;
        } else {
            this.targetArmAngle = Constants.Reef.coralEjectAngleLevel4;
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer = new Timer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        timer.start();
        // set arm to correct angle
        current_angle = Robot.wristSubsystem.getWristPosition();
        // Calculate power curve proportional
        double armRotationPower = Math.abs(this.targetArmAngle - current_angle) / 100;
        // Move arm up or down to target arm angle
        if (Math.abs(this.targetArmAngle - current_angle) > Constants.Tolerances.coralEjectAngleTolerance) {
            double v_sign = Math.signum(this.targetArmAngle - current_angle);
            Robot.wristSubsystem.setWristSpeed(v_sign * (armRotationPower));
        } else {
            Robot.wristSubsystem.setWristSpeed(0d);
            Robot.gamePieceManipulator.ejectCoral();
            if (timer.hasElapsed(Constants.Times.coralEjectMotorRunTime)) {
                isDone = true;
                isFinished();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        Robot.gamePieceManipulator.stopCoralMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
