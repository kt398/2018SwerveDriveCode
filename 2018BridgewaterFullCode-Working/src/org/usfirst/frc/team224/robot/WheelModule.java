package org.usfirst.frc.team224.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.TalonSRX.FeedbackDevice;
//import com.ctre.TalonSRX.TalonControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class WheelModule {

	WPI_TalonSRX steer;// motor to steer.
	WPI_TalonSRX drive;// motor to drive.
	private int whichWheel;// number of the wheel
	private int invert;
	private int sideInverted;
	private double currentAngle;
	/*
	 * n iinhn nh hright side has the motor direction inverted, so this has to be -1
	 * for the right side.
	 */

	public WheelModule(int whichModule) {
		whichWheel = whichModule;
		drive = new WPI_TalonSRX(whichModule + 1);
		drive.setInverted(false);
		/* first choose the sensor */
		drive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);
		drive.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		drive.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		drive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* set the peak and nominal outputs */
		drive.configNominalOutputForward(0, Constants.kTimeoutMs);
		drive.configNominalOutputReverse(0, Constants.kTimeoutMs);
		drive.configPeakOutputForward(1, Constants.kTimeoutMs);
		drive.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		drive.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		drive.config_kF(0, 0.4, Constants.kTimeoutMs);// .2
		drive.config_kP(0, 0.4, Constants.kTimeoutMs);// .2
		drive.config_kI(0, 0, Constants.kTimeoutMs);
		drive.config_kD(0, 0, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		drive.configMotionCruiseVelocity(8000, Constants.kTimeoutMs);// 5000
		drive.configMotionAcceleration(4000, Constants.kTimeoutMs);// 2000
		/* zero the sensor */
		drive.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/*
		 * 
		 * 
		 *
		 *
		 * SPACER BETWEEN STEER AND DRIVE
		 * 
		 * 
		 * 
		 * 
		 * 
		 * 
		 * 
		 */
		steer = new WPI_TalonSRX(whichModule);
		steer.setSensorPhase(false);
		steer.setInverted(true);
		steer.config_kP(0, Constants.steerP, 0);
		steer.config_kI(0, Constants.steerI, 0);
		steer.config_kD(0, Constants.steerD, 0);
		steer.configNominalOutputForward(0, 0);
		steer.configNominalOutputReverse(0, 0);
		steer.configPeakOutputForward(1, 0);
		steer.configPeakOutputReverse(-1, 0);
		steer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
	}

	public void drive(double input, double speed) {
		double newAngle;
		double inputAngle = steerConvertToEncoderTicks(input);
		double currentAngle = steer.getSelectedSensorPosition(0);
		double invertedInput = (inputAngle + 2048) % 4096;
		if (Math.abs(Math.IEEEremainder(inputAngle - currentAngle, 4096)) <= Math
				.abs(Math.IEEEremainder(invertedInput - currentAngle, 4096))) {
			newAngle = currentAngle + Math.IEEEremainder(inputAngle - currentAngle, 4096);
			invert = 1;
		} else {
			invert = -1;
			newAngle = currentAngle + Math.IEEEremainder(invertedInput - currentAngle, 4096);
		}

		// steer.set(steerConvertToEncoderTicks(newAngle));
		steer.set(ControlMode.Position, newAngle);
		drive.set(ControlMode.PercentOutput, speed * invert * sideInverted);
	}

	public void driveDistanceMotionMagic(double encoderDistance, double angle) {
		drive.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		drive.set(ControlMode.MotionMagic, encoderDistance * getInverted());
	}

	public void driveDistanceNavX(double speed, double angle) {

		drive.set(ControlMode.PercentOutput, speed * invert * sideInverted);
	}

	public void turn(double angle) {
		this.drive(angle, 0);
	}

	public double steerConvertToDegrees(double encoderTicks) {
		// based on ratio
		return ((360 / 4096) * encoderTicks) % 360;
	}

	public double steerConvertToEncoderTicks(double angle) {
		// based on ratio
		return 4096 / 360.0 * angle;

	}

	public void disableDrive() {
		drive.disable();
	}

	public void disableSteer() {
		steer.disable();
	}

	public void disableBoth() {
		disableSteer();
		disableDrive();
	}

	public void print() {
		SmartDashboard.putNumber("Wheel " + whichWheel + " angle:", (steer.getSelectedSensorPosition(0)));
	}

	public void resetDriveEncPos() {
		drive.setSelectedSensorPosition(0, 0, 0);
	}

	public double getDriveEncPos() {
		return drive.getSelectedSensorPosition(0);
	}

	public double getSteerEncPos() {
		return steer.getSelectedSensorPosition(0);
	}

	public void matchStartup() {

	}

	public void resetSteerEncPos() {
		steer.setSelectedSensorPosition(0, 0, 0);
	}

	private int getInverted() {
		if (invert * sideInverted > 0) {
			return 1;
		}
		return -1;
	}

	public double getError() {
		return steer.getClosedLoopError(0);
	}

	public boolean inverted() {
		if (getInverted() == 1) {
			return false;
		}
		return true;
	}

	public void init() {

		switch (whichWheel) {
		case 0:
			steer.setSelectedSensorPosition((int) (-steer.getSensorCollection().getPulseWidthPosition() - 1240), 0, 0);
			break;
		case 2:
			steer.setSelectedSensorPosition((int) (-steer.getSensorCollection().getPulseWidthPosition() - 998), 0, 0);
			break;
		case 4:
			steer.setSelectedSensorPosition((int) (-steer.getSensorCollection().getPulseWidthPosition() - 2734), 0, 0);
			break;
		case 6:
			steer.setSelectedSensorPosition((int) (-steer.getSensorCollection().getPulseWidthPosition() - 3049), 0, 0);
			break;
		}
		// TRUST ME ITS MINUS MINUS
		// steer.setSelectedSensorPosition(0, 0, 0); NO LONGER NEED THIS LINE
		steer.configAllowableClosedloopError(0, 0, 0);
		if (whichWheel >= 4) {
			sideInverted = -1;
		} else {
			sideInverted = 1;
		}
	}
}