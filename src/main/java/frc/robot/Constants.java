package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
                new PIDConstants(3.14, 0, 0.3),
                new PIDConstants(3.14, 0, 0.3),
                MAX_SPEED,
                14.778574017813,
                new ReplanningConfig(false, true)
            );
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
    }
}
