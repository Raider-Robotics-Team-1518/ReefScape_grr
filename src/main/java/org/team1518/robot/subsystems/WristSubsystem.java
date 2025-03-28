// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1518.robot.Constants;

public class WristSubsystem extends SubsystemBase {

    private TalonFX wristMotor;
    private double coralArmPosition = 0;
    private Encoder wristEncoder = new Encoder(2, 3);

    public WristSubsystem() {
        wristMotor = new TalonFX(Constants.Motors.wristMotorId);
        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralModeValue.Brake);
        wristMotor.setPosition(0);
    }

    public void setWristSpeed(double speed) {
        // positive for rotating towards a more vertical angle
        double currentWristPosition = getWristPosition();
        if (
            (speed > 0 && currentWristPosition < Constants.Limits.wristMaxAngle) ||
            (speed < 0 && currentWristPosition > Constants.Limits.wristMinAngle)
        ) {
            wristMotor.set(speed);
        } else {
            stopWrist();
        }
    }

    public void stopWrist() {
        wristMotor.set(0);
    }

    public double getWristPosition() {
        // get the angle of the coral arm/wrist/whatever we're going to call it
        coralArmPosition = wristEncoder.get();
        //coralArmPosition /= 100; // divide by gearbox ratio
        return coralArmPosition * Constants.Factors.wristDegreesPerEncoderCount; // degrees, but should it be radians?*/
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Wrist Position", getWristPosition());
    }
}
