// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
// import org.team1518.robot.Constants;
import org.team1518.robot.LimeLight;
import org.team1518.robot.LimelightHelpers;
import org.team1518.robot.LimelightHelpers.LimelightTarget_Fiducial;
import org.team1518.robot.Utils;

public class Blinkies extends SubsystemBase {

    public LEDState lightState = LEDState.DEFAULT;
    public final DigitalOutput dOutput1 = new DigitalOutput(6);
    public final DigitalOutput dOutput2 = new DigitalOutput(7);
    public final DigitalOutput dOutput3 = new DigitalOutput(8);
    public final DigitalOutput dOutput4 = new DigitalOutput(9);
    private LimelightTarget_Fiducial llAprilTag = new LimelightHelpers.LimelightTarget_Fiducial();
    private LimeLight limelight = new LimeLight();

    public Blinkies() {
        limelight.init("limelight");
    }

    @Override
    public void periodic() {
        int fID = (int) LimelightHelpers.getFiducialID("limelight");
        SmartDashboard.putNumber("tx", llAprilTag.tx);
        SmartDashboard.putNumber("fID", fID);

        if (RobotState.isDisabled()) {
            setLEDState(LEDState.DEFAULT);
        } else {
            Optional<Alliance> allianceColorOpt = DriverStation.getAlliance();
            if (allianceColorOpt.isPresent()) {
                Alliance allianceColor = allianceColorOpt.get();
                if (allianceColor == Alliance.Red) {
                    // do stuff if we're on the red alliance
                    setLEDState(LEDState.RED);
                } else if (allianceColor == Alliance.Blue) {
                    // do stuff if we're on the blue alliance
                    setLEDState(LEDState.BLUE);
                }

                SmartDashboard.putString("alliance color", allianceColor.toString());
                if (fID > 0) {
                    double distance = limelight.getDistanceToTarget();
                    SmartDashboard.putNumber("Distance", Utils.round2prec(distance, 1));

                    if (distance > 0) {
                        // if centered left/right on the april tag
                        setLEDState(LEDState.GREEN);
                    } else {
                        // SmartDashboard.putString("LED Color", "black");
                    }
                    // } else {
                    // SmartDashboard.putString("LED Color", "white");
                }
                // } else {
                // SmartDashboard.putString("LED Color", "pink");
            }
        }
        /*
         * } else if (note is loaded) {
         * setLEDState(LEDState.ORANGE)
         * }
         */
    }

    public void setLEDState(LEDState state) {
        dOutput1.set(state.do1);
        dOutput2.set(state.do2);
        dOutput3.set(state.do3);
        this.lightState = state;
    }

    public enum LEDState {
        // the state of the DIO ports must match the code on the LED controller
        // which is looking for a pattern of highs & lows to determine the
        // color or pattern to show
        OFF(false, false, false), // 0
        RED(true, false, false), // 1
        BLUE(false, true, false), // 2
        GREEN(true, true, false), // 3
        ORANGE(false, false, true), // 4
        YELLOW(true, false, true), // 5
        PURPLE(false, true, true), // 6
        DEFAULT(true, true, true); // 7

        private final boolean do1, do2, do3;

        private LEDState(boolean do1, boolean do2, boolean do3) {
            this.do1 = do1;
            this.do2 = do2;
            this.do3 = do3;
        }
    }
}
