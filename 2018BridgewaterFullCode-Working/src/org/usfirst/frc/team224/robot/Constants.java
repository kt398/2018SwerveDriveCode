package org.usfirst.frc.team224.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Constants {

	// Gear ratio between drive encoder and drive wheel
	public static final double driveWheelRatio = 30.0 / 48;
	// Gear ratio between steer encoder and steer wheel
	public static final double steerWheelRatio = 1;
	// P value for steering motor
	public static final double steerP = 2;
	// I value for steering motor
	public static final double steerI = 0.001;
	// D value for steering motor
	public static final double steerD = 0.1;
	// P value for driving motor
	public static final double driveP = Double.NaN;
	// I value for driving motor
	public static final double driveI = Double.NaN;
	// D value for driving motor
	public static final double driveD = Double.NaN;
	// Diameter of drive wheel
	public static final double wheelDiameter = 3.25;
	// Encoder value of the elevator to pick up cubes from the ground
	public static final double elevatorDownEncoderValue = 0;
	// Encoder value of the elevator to place cubes in switch
	public static final double elevatorMiddleEncoderValue = -24500;
	// Encoder value of the elevator to place cubes on sclale
	public static final double elevatorTopEncoderValue = -81800;
	// No idea what this is
	public static final double wheelSpinRadius = 0;
	// Distance along the length of the robot between centers of the two wheels
	public static final double wheelbase = 22.5;
	// Distance along the width of the robot between middle of the treads of two
	// wheels
	public static final double trackwidth = 27;
	// Encoder ticks for 1 shaft revolution, 4096 for CTRE encoder (probably)
	public static final int encoderTicks = 4096;

	public static final int kSlotIdx = 0;

	public static final int kPIDLoopIdx = 0;
	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;
	public static final Value raiserDown = Value.kForward;
	public static final Value raiserUp = Value.kReverse;
	public static final Value openArms = Value.kForward;
	public static final Value closeArms = Value.kReverse;
	public static final Value pushOutPusher = Value.kReverse;
	public static final Value pullInPusher = Value.kForward;
	public static final double elevatorP = .5;

	public static final double elevatorI = 0;

	public static final double elevatorD = 0;

}