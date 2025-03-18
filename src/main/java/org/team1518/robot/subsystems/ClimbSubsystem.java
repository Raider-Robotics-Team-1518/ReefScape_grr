// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1518.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private SparkMax climbMotor = new SparkMax(Constants.Motors.climbMotorId, MotorType.kBrushless);
    private SparkBaseConfig climbConfig;

    //private double elevatorPosition = 0;
    //private Encoder elevatorEncoder = new Encoder(0, 1);

    /** Creates a new ClimbSubsystem. */
    public ClimbSubsystem() {
        climbConfig = new SparkMaxConfig();
        climbConfig.idleMode(IdleMode.kBrake);
        //climbConfig.inverted(true);
        climbMotor.configure(climbConfig, null, null);
    }

    public void runClimbMotor(double speed) {
        climbMotor.set(speed);
    }

    public void stopClimbMotor() {
        climbMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
