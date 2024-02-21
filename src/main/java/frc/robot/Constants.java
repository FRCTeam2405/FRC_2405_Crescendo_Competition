package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {

    public static final class Swerve {
        /** Swerve drive max speed, in meters per second. */
        public static final double MAX_SPEED = 0.5;
        /** Swerve drive max angular speed, in radians per second. */
        public static final double MAX_ANGULAR_SPEED = 0.05 * (2 * Math.PI);

        //TODO! Configure the path follower
        public static final HolonomicPathFollowerConfig PATH_PLANNER_CONFIG = 
            new HolonomicPathFollowerConfig(
                // Best Values So Far: kP: 3.14, kI: 0, kD: 0.1
                /*
                new PIDConstants(4.5, 0, 0.05),
                new PIDConstants(3.14, 0, 0.1),
                */
                new PIDConstants(1, 0, 0.01),
                new PIDConstants(1, 0, 0.01),
                MAX_SPEED,
                14.778574017813,
                new ReplanningConfig(false, true)
            );
    }

    public static final class Intake {

        public static final class Motors {

            public static final int RIGHT_INTAKE_PORTID = 45;

            public static final boolean RIGHT_INTAKE_INVERTED = true;

            public static final double RIGHT_INTAKE_SPEED_MAX = 0.70;
        }
    }

    public static final class Shooter {
        //TODO! adjust for shooter
        public static final double SHOOTER_HEIGHT = 0;

        public static final class Motors {

            public static final int TOP_SHOOTER_PORTID = 43;
            public static final int BOTTOM_SHOOTER_PORTID = 41;

            public static final boolean TOP_SHOOTER_INVERTED = false;
            public static final boolean BOTTOM_SHOOTER_INVERTED = false;

            public static final double TOP_SHOOTER_PERCENT_OUTPUT_MAX = 0.75;
            public static final double BOTTOM_SHOOTER_PERCENT_OUTPUT_MAX = 0.75;

            public static final double TOP_SHOOTER_VELOCITY_MAX = 6500;
            public static final double BOTTOM_SHOOTER_VELOCITY_MAX = 6500;

            public static final double TOP_SHOOTER_VELOCITY_DEFAULT = 5000;
            public static final double BOTTOM_SHOOTER_VELOCITY_DEFAULT = 5000;
        }

        public static final class Encoder {

            public static final double GAIN_P = 0.0005;
            public static final double GAIN_I = 0.0;
            public static final double GAIN_D = 0.0;
            public static final double ZONE_I = 0.0;
            public static final double FEED_FORWARD = 0.000182;
            public static final double OUTPUT_RANGE_MIN = -1.0;
            public static final double OUTPUT_RANGE_MAX = 1.0;
        }
    }

    public static final class Feeder {

        public static final class Sensors {

            public static final int NOTE_LIMIT_PORTID = 0;
        }
        
        public static final class Motors {

            public static final int TOP_FEEDER_PORTID = 42;
            public static final int BOTTOM_FEEDER_PORTID = 44;

            public static final boolean TOP_FEEDER_INVERTED = true;
            public static final boolean BOTTOM_FEEDER_INVERTED = true;

            public static final double TOP_FEEDER_SPEED_MAX = 0.75;
            public static final double BOTTOM_FEEDER_SPEED_MAX = 0.75;
        }
    }

    public static final class Arm {

        public static final class Motors {
            public static final int ARM_PORTID = 40;

            public static final boolean ARM_INVERTED = false;

            public static final double ARM_SPEED_MAX = 0.50;
            public static final double ARM_VELOCITY = 30;
            public static final double ARM_VELOCITY_MAX = 40;
        }

        public static final class Encoder {

            public static final class ProfileZero {

                public static final int PROFILE_ID = 0;
                public static final double GAIN_P = 0.005;

            }
        }

        public static final class SetPoints {

            public static final double HOME = 0.0;
            public static final double AMP = 150.0;
            public static final double OVERRIDE = 50;
        }
    }

    public static final class Field {
        public static final double BLUE_SPEAKER_X = 0;
        public static final double BLUE_SPEAKER_Y = 5.547879;
        public static final double BLUE_SPEAKER_Z = 2.032004;

        public static final double RED_SPEAKER_X = 16.57938;
        public static final double RED_SPEAKER_Y = 5.547879;
        public static final double RED_SPEAKER_Z = 2.032004;

    }

    public static final class Controllers {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;

        public static final class Taranis {
            // Port IDs for the Taranis driver controller
            // These should match the Xbox controller layout
            public static final int DRIVE_X_AXIS = 1;
            public static final int DRIVE_Y_AXIS = 0;
            public static final int ROTATE_AXIS = 4;

            public static final double DRIVE_DEADBAND = 0.05;
            public static final double ROTATE_DEADBAND = 0.05;

            public static final int ZERO_GYRO_BUTTON = 7;

            public static final int ROTATE_90_DEGREES_BUTTON = 5;

            public static final int ROTATE_TO_APRILTAG_BUTTON = 4;

            public static final int INTAKE_NOTE_BUTTON = 1;
            public static final int FIRE_WHEN_READY_BUTTON = 2; //To be mapped to guitar
            public static final int MOVE_ARM_TO_AMP = 3;
            public static final int MOVE_ARM_TO_HOME = 6;
        }

        public static final class Guitar {
            // Port IDs for the buttons on the controller
            public static final int GREEN_FRET = 3;
            public static final int RED_FRET = 2;
            public static final int YELLOW_FRET = 1;
            public static final int BLUE_FRET = 0;
            public static final int ORANGE_FRET = 6;

            public static final int START_BUTTON = 9;
            public static final int SELECT_BUTTON = 8;

            // Port IDs for the axes on the controller (joystick)
            public static final int JOYSTICK_X = 0;
            public static final int JOYSTICK_Y = 1;

            // Angles for the POV system on the controller (strum bar)
            public static final int STRUM_UP = 0;
            public static final int STRUM_DOWN = 90;
            public static final int STRUM_NEUTRAL = -1;
        }

        public static final class AprilTags {

            public static final int BLUE_SOURCE_RIGHT = 1;
            public static final int BLUE_SOURCE_LEFT = 2;
            public static final int RED_SPEAKER_RIGHT = 3;
            public static final int RED_SPEAKER_CENTER = 4;
            public static final int RED_AMP = 5;
            public static final int BLUE_AMP = 6;
            public static final int BLUE_SPEAKER_CENTER = 7;
            public static final int BLUE_SPEAKER_LEFT = 8;
            public static final int RED_SOURCE_RIGHT = 9;
            public static final int RED_SOURCE_LEFT = 10;
            public static final int RED_STAGE_LEFT = 11;
            public static final int RED_STAGE_RIGHT = 12;
            public static final int RED_STAGE_CENTER = 13;
            public static final int BLUE_STAGE_CENTER = 14;
            public static final int BLUE_STAGE_RIGHT = 15;
            public static final int BLUE_STAGE_LEFT = 16;
        }
    }

    public static final class Dashboard {

        public static final class Main {
            public static final String TAB_NAME = "Main";
        }

        public static final class Utility {
            public static final String TAB_NAME = "Utility";

            public static final class Widgets {
                public static final String TOP_SHOOTER_VELOCITY_SETTING_NAME = "Top Shooter Velocity Setting";
                public static final String BOTTOM_SHOOTER_VELOCITY_SETTING_NAME = "Bottom Shooter Velocity Setting";
                public static final String TOP_SHOOTER_VELOCITY_NAME = "Top Shooter Velocity";
                public static final String BOTTOM_SHOOTER_VELOCITY_NAME = "Bottom Shooter Velocity";

                public static final String TOP_FEEDER_OUTPUT_SETTING_NAME = "Top Feeder Output Setting";
                public static final String BOTTOM_FEEDER_OUTPUT_SETTING_NAME = "Bottom Feeder Output Setting";
                
                public static final String RIGHT_INTAKE_OUTPUT_SETTING_NAME = "Right Intake Output Setting";
                
                public static final String ARM_VELOCITY_SETTING_NAME = "Arm Velocity Setting";
                public static final String ARM_POSITION_NAME = "Arm Position";
            }
        }
    }
}
