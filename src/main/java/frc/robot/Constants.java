// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkMaxPIDController.AccelStrategy;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.2;
		public static final double kTriggerDeadzone = .05;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			public static final int kSquare = 1;
			public static final int kX = 2;
			public static final int kCircle = 3;
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

	public static final class DriveConstants {
		// CAN IDs (updated)
		public static final int kCounterWeightPort = 17;
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;
		public static final double kDriveScale = 0.5;
		// Drive PID values
		public static final double kP = 0.005;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;
		public static final AccelStrategy kTrapezoidal = AccelStrategy.kTrapezoidal;
		public static final int kSlotID = 0;
		public static final double kMaxAcel = 0;
		public static final double kMaxVelocity = 2;
		public static final double kAllowedError = 0;
		public static final double kMinVelocity = 0;
		/*** Distance between center of front wheel and center of back wheel */
		public static final double kWheelBase = 21.5;
		/*** Distance between center of left wheel and center of right wheel */
		public static final double kTrackWidth = 21.5;
		public static final double kSteerPeriod = 0.02;
		public static final boolean kFrontLeftDriveInverted = true;
		public static final boolean kBackLeftDriveInverted = true;
		public static final boolean kFrontRightDriveInverted = false;
		public static final boolean kBackRightDriveInverted = false;
		// Speed multiplier to make sure the robot doesn't crash into something when
		// testing, because crashing into people's shins would be bad
		public static final double kSpeedMultiplier = 0.25;
	}

	public static final class SwerveConstants {
		public static final double gearRatio = 8.14;
		public static final double wheelDiameter = 0.1016;  // in meters
		// public static final double ticksPerAxisRev = 42;

        public static final double kTicksToMeters = (1/gearRatio) * Math.PI * wheelDiameter;
		public static final double kMotorRevsPerMeter = gearRatio/(Math.PI * wheelDiameter);


		// These set points resulted in back being front, front is back
		public static final double FrontLeftZero = 124.89;
		public static final double FrontRightZero = 115.66;
		public static final double BackLeftZero = 277.47;
		public static final double BackRightZero = 212.73;

		// These encoder set points should be correct
		// public static final double FrontLeftZero = -56.89;
		// public static final double FrontRightZero = -65.66;
		// public static final double BackLeftZero = 97.47;
		// public static final double BackRightZero = 32.73;


	}
}
