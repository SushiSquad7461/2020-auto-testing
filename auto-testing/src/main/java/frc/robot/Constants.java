// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        // drive motor ids
        public static final int fR_ID = 0;
        public static final int fL_ID = 1;
        public static final int bR_ID = 2;
        public static final int bL_ID = 3;

        // controller scaling
        public static final double CONTROLLER_LINEAR_SCALING = 75;
        public static final double CONTROLLER_ANGULAR_SCALING = 100;
        public static final double CONTROLLER_QUICKTURN_SCALING = 400;

        // pid constants
		public static final double LEFT_kP = 0.356;
		public static final double LEFT_kI = 0;
		public static final double LEFT_kD = 0;

		public static final double LEFT_kS = 0.166;
		public static final double LEFT_kV = 0.0315;
		public static final double LEFT_kA = 0.00743;

		public static final double RIGHT_kP = 0.356;
		public static final double RIGHT_kI = 0;
		public static final double RIGHT_kD = 0;

		public static final double RIGHT_kS = 0.166;
		public static final double RIGHT_kV = 0.0315;
		public static final double RIGHT_kA = 0.00743;
		
		// closed loop constants
		public static final double wheelRadius = 3.0;
		public static final double trackWidth = 0.6420;

    }
}
