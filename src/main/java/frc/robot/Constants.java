package frc.robot;

public final class Constants {
    public static final class Swerve {
        /** Swerve drive max speed, in meters per second. */
        public static final double MAX_SPEED = 4.0;
    }

    public static final class Controllers {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;

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
