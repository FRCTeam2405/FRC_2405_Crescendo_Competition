package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {

    public static final class Swerve {
        /** Swerve drive max speed, in meters per second. */
        public static final double MAX_SPEED = 0.95;
        /** Swerve drive max angular speed, in radians per second. */
        // Max speed: ~1.5 rotations per second
        public static final double MAX_ANGULAR_SPEED = 0.4 * (2 * Math.PI);

        public static final HolonomicPathFollowerConfig PATH_PLANNER_CONFIG = 
            new HolonomicPathFollowerConfig(
                // Best Values So Far: kP: 3.14, kI: 0, kD: 0.1
                /*
                new PIDConstants(4.5, 0, 0.05),
                new PIDConstants(3.14, 0, 0.1),
                */
                new PIDConstants(4.5, 0, 0.06),
                new PIDConstants(5.5, 0, 0.01),
                4.5,
                0.4,
                new ReplanningConfig(false, true)
            );
    }

    public static final class Intake {

        public static final class Motors {

            public static final int RIGHT_INTAKE_PORTID = 45;

            public static final boolean RIGHT_INTAKE_INVERTED = true;

            public static final double RIGHT_INTAKE_SPEED_MAX = 0.70;
            public static final double RIGHT_INTAKE_REVERSE_SPEED_MAX = -0.70;
        }
    }

    public static final class Auton {

        public static final class startPose {

            public static final double BLUE_1_ROTATION = 1.03777277;

            public static final double BLUE_2_ROTATION = 0;

            public static final double BLUE_3_ROTATION = -1.04335783;

            public static final double BLUE_1_X = 0.71;

            public static final double BLUE_2_X = 1.38;

            public static final double BLUE_3_X = 0.74;

            public static final double BLUE_1_Y = 6.71;

            public static final double BLUE_2_Y = 5.55;

            public static final double BLUE_3_Y = 4.38;

            public static final double RED_1_ROTATION = 2.115688119;

            public static final double RED_2_ROTATION = 3.1415926535897;

            public static final double RED_3_ROTATION = -2.07606914;

            public static final double RED_1_X = 15.86938;

            public static final double RED_2_X = 15.19938;

            public static final double RED_3_X = 15.83938;

            public static final double RED_1_Y = 6.71;

            public static final double RED_2_Y = 5.55;

            public static final double RED_3_Y = 4.53;
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

            public static final double TOP_SHOOTER_VELOCITY_ALTERNATE = 3500; 
            public static final double BOTTOM_SHOOTER_VELOCITY_ALTERNATE = 1600;

            public static final double TOP_SHOOTER_VELOCITY_AMP = 1000; 
            public static final double BOTTOM_SHOOTER_VELOCITY_AMP = 1000;
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

            public static final double TOP_FEEDER_SHOOTING_SPEED = 0.75;
            public static final double BOTTOM_FEEDER_SHOOTING_SPEED = 0.75;

            public static final double TOP_FEEDER_INTAKING_SPEED = 0.20;
            public static final double BOTTOM_FEEDER_INTAKING_SPEED = 0.20;

            public static final double REVERSE_FEEDER_INTAKING_SPEED = -0.25;
            ;
        }
    }

    public static final class Arm {

        public static final double DIRECT_DRIVE_MOD = 20.0;

        public static final class Motors {
            public static final int ARM_PORTID = 40;

            public static final boolean ARM_INVERTED = true;

            public static final double ARM_SPEED_MAX = 0.50;
            public static final double ARM_VELOCITY = 30;
            public static final double ARM_VELOCITY_MAX = 40;
        }

        public static final class Encoder {

            public static final class ProfileZero {

                public static final int PROFILE_ID = 0; // Volts
                public static final double GAIN_P = 2.4;
                public static final double GAIN_I = 0;
                public static final double GAIN_D = 0.1;
                public static final double GAIN_FF_ACCELERATION = 0;
                public static final double PEAK_FORWARD_VOLTAGE = 8;
                public static final double PEAK_REVERSE_VOLTAGE = -8;

            }
        }

        public static final class SetPoints {

            public static final double HOME = 0.0;
            public static final double AMP = 100.0; // 150, original setting
            public static final double CLIMB = 120.0;
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

            public static final int LED_LIGHTS_TEST_BUTTON = 6;

            public static final int ZERO_GYRO_BUTTON = 7;

            public static final int ROTATE_90_DEGREES_BUTTON = 5;

            public static final int ROTATE_TO_SPEAKER_BUTTON = 4;

            public static final int ADD_VISION_MEASURMENT_BUTTON = 8;

            public static final int INTAKE_NOTE_BUTTON = 1;
            public static final int REVERSE_INTAKE_NOTE_BUTTON = 2;
        }

        public static final class Guitar {
            // Port IDs for the buttons on the controller
            public static final int GREEN_FRET = 1;
            public static final int RED_FRET = 2;
            public static final int YELLOW_FRET = 4;
            public static final int BLUE_FRET = 3;
            public static final int ORANGE_FRET = 5;

            public static final int OPTION_A_BUTTON = 7;
            public static final int OPTION_B_BUTTON = 8;

            // Port IDs for the axes on the controller (joystick)
            public static final int JOYSTICK_X = 0;
            public static final int JOYSTICK_Y = 1;
            public static final int ROBOT_EMOTION_ID = 2;

            public static final double X_DEADBAND = 0.05;
            public static final double Y_DEADBAND = 0.05;

            public static final boolean X_INVERTED = true;
            public static final boolean Y_INVERTED = true;

            // Angles for the POV system on the controller (strum bar)
            public static final int STRUM_UP = 0;
            public static final int STRUM_DOWN = 180;
            public static final int STRUM_NEUTRAL = -1;

            public static final class ROBOT_EMOTION_SETTING {

                public static final double SADNESS = 0.1;
                public static final double FEAR = 0.3;
                public static final double JOY = 0.5;
                public static final double DISGUST = 0.7;
                public static final double ANGER = 0.9;
            }
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

    public static final class LEDs {
        //we have 2 sets of LED lights, meaning we're going to need to set both in each command
        public static final int LED_PORT_ONE = 0;
        public static final int LED_PORT_TWO = 1;

        public static final class LED_COLORS {

            //These are imported colors from last season.
            //This color will be what the LEDs set to when the robot is turned on
            public static final double TELEOP_COLOR_ONE_DEFAULT = -0.95;
            public static final double TELEOP_COLOR_TWO_DEFAULT = -0.91;

            public static final double INTAKE_COLOR_ONE = -0.29;
            public static final double INTAKE_COLOR_TWO = -0.15;

            public static final double INTAKE_REVERSE_COLOR_ONE = -0.31;
            public static final double INTAKE_REVERSE_COLOR_TWO = -0.17;

            public static final double SHOOTER_COLOR_ONE = -0.21;
            public static final double SHOOTER_COLOR_TWO = -0.19;

            public static final double STROBE_RED = -0.11;
            public static final double HEARTBEAT_RED = -0.25;
            public static final double SOLID_RED = 0.61;
            public static final double SHOT_BLUE = -0.83;
            public static final double GREEN = 0.77;
            public static final double YELLOW = 0;
            public static final double AQUA = 0.81;
            public static final double BLUE = 0.87;
        }

        public static final class LED_ACTIONS {
            public static final double INTAKE_INVALID = -0.11;
        }

        public static final class ROBOT_EMOTIONS {

            public static final double SADNESS_COLOR_ONE = -0.41;
            public static final double SADNESS_COLOR_TWO = -0.65;

            public static final double FEAR_COLOR_ONE = -0.27;
            public static final double FEAR_COLOR_TWO = -0.29;

            public static final double JOY_COLOR_ONE = -0.87;
            public static final double JOY_COLOR_TWO = -0.89;

            public static final double DISGUST_COLOR_ONE = 0.07;
            public static final double DISGUST_COLOR_TWO = -0.27;

            public static final double ANGER_COLOR_ONE = -0.57;
            public static final double ANGER_COLOR_TWO = -0.85;
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

                public static final String TOP_FEEDER_SHOOTING_OUTPUT_SETTING_NAME = "Top Feeder Shooting Output Setting";
                public static final String BOTTOM_FEEDER_SHOOTING_OUTPUT_SETTING_NAME = "Bottom Feeder Shooting Output Setting";
                
                public static final String TOP_FEEDER_INTAKING_OUTPUT_SETTING_NAME = "Top Feeder Intaking Output Setting";
                public static final String BOTTOM_FEEDER_INTAKING_OUTPUT_SETTING_NAME = "Bottom Feeder Intaking Output Setting";
                
                public static final String FEEDER_NOTE_LIMIT_NAME = "Feeder Note Limit";

                public static final String RIGHT_INTAKE_OUTPUT_SETTING_NAME = "Right Intake Output Setting";
                
                public static final String ARM_VELOCITY_SETTING_NAME = "Arm Velocity Setting";
                public static final String ARM_POSITION_SETTING_NAME = "Arm Position Setting";
                public static final String ARM_POSITION_NAME = "Arm Position";

                public static final String ROBOT_EMOTION_SETTING_NAME = "Robot Emotion Setting";

                public static final String PHANTOM_NAME = "Dont look here";
            }
        }

        public static final class MotorControl {

            public static final String TAB_NAME = "Motor Control";

            public static final class Widgets {

                
            }
        }
    }
}
