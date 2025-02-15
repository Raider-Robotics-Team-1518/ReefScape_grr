package org.team1518.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kVoltage = 12.0;

    public static final int kDriver = 0;
    public static final int kCoDriver = 1;

    /**
     * The RobotMap class defines CAN IDs, CAN bus names, DIO/PWM/PH/PCM channel
     * IDs, and other relevant identifiers for addressing robot hardware.
     */
    public static final class RobotMap {

        public static final String kSwerveCANBus = "rio";

        public static final int kFlMove = 10;
        public static final int kFlTurn = 4;
        public static final int kFrMove = 9;
        public static final int kFrTurn = 2;
        public static final int kBlMove = 5;
        public static final int kBlTurn = 6;
        public static final int kBrMove = 7;
        public static final int kBrTurn = 8;

        public static final int kFlEncoder = 12;
        public static final int kFrEncoder = 15;
        public static final int kBlEncoder = 13;
        public static final int kBrEncoder = 14;

        public static final int kCanandgyro = 20;
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class Motors {

        public static final int elevatorMotorId = 26;
        public static final int wristMotorId = 17;
        public static final int gamePieceMotorId = 18;
    }

    public static class Limits {

        public static final double elevatorMax = 9500;
        public static final double elevatorMin = 0;
        public static final double wristMinAngle = 0;
        public static final double wristMaxAngle = 180;
    }

    public static class Factors {

        public static final double elevatorInchesPerRevolution = 1;
        public static final double wristDegreesPerRevolution = 360;
    }

    public static class Reef {

        // Per Q&A system: "Keep in mind that dimensions on the REEF
        // and other structures have a tolerance of +/- 1/2 in. to
        // accommodate variances in manufacturing and assembly."
        public static final double[] levels = {
            0, // we ignore this level, leave at 0
            18, // height in inches of level 1 of the reef
            31.875, // level 2
            47.625, // level 3
            72 // level 4
        };
        public static final double coralIntakeHeight = 16;
        // optimal angle in degrees to eject coral onto level 1
        public static final double coralEjectAngleLevel1 = 10; // nearly horizontal
        public static final double coralEjectAngleLevel23 = 30;
        public static final double coralEjectAngleLevel4 = 90; // vertical
        // optimal angle in degrees for ejecting algae into the processor
        public static final double algaeEjectArmAngle = 20;
        public static final double targetCoralIntakeAngle = 70;
        public static final double targetAlgaeIntakeAngle = 20;
    }

    public static final class Tolerances {

        public static final double reefHeightTolerance =1.0; // tolerance for height of the reef levels
        public static final double coralEjectAngleTolerance = 0.1; // angle of coral manipulator arm
        public static final double coralIntakeAngleTolerance = 0.1;
        public static final double algaeEjectAngleTolerance = 0.1;
        public static final double algaeIntakeAngleTolerance = 0.1;
        public static final double minElevatorHeightTolerance = 1.0;
    }

    public static final class MotorSpeeds {

        public static final double elevatorPower = 0.25;
        // TODO: either intake or eject need to have negative values
        public static final double coralEjectMotorSpeed = 0.5;
        public static final double coralIntakeMotorSpeed = 0.25;
        public static final double algaeEjectMotorSpeed = 0.5;
        public static final double algaeIntakeMotorSpeed = 0.5;
    }

    public static final class Times {

        public static final double coralEjectMotorRunTime = 2.5;
        public static final double coralIntakeMotorRunTime = 10;
        public static final double algaeMotorRunTime = 2.5;
    }

    public static final class ColorValues {

        // 95% value would be a very light version of any color
        public static final float whiteValueMin = 95f;
        // For HSL, define the min & max of HUE that is the target color
        // see https://hslpicker.com/#ff6a00
        public static final float whiteHueMin = 30.0f;
        public static final float whiteHueMax = 80.0f;
    }
}
