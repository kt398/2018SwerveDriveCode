package org.usfirst.frc.team224.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
	public WheelModule frontLeft;
	private WheelModule backLeft;
	private WheelModule frontRight;
	private WheelModule backRight;
	private Joystick joy;
	private double fwd, str, rcw;
	private double l, w, r;
	public AHRS navx;
	private double a, b, c, d;
	private double ws0, ws1, ws2, ws3;
	private double wa0, wa1, wa2, wa3;
	private double throttle;
	private double max;
	private Timer t;

	public SwerveDrive(Joystick joy) {
		t = new Timer();
		frontLeft = new WheelModule(0);
		backLeft = new WheelModule(2);
		frontRight = new WheelModule(4);
		backRight = new WheelModule(6);
		navx = new AHRS(SPI.Port.kMXP);
		navx.reset();
		this.joy = joy;
		l = Constants.wheelbase;
		w = Constants.trackwidth;
		r = Math.sqrt(Math.pow(l, 2) + Math.pow(w, 2));
	}

	public void setAngleAdjustment(int angle) {
		navx.setAngleAdjustment(angle);
	}

	public void drive() {
		calculations();
		if ((Math.abs(joy.getMagnitude()) > .2 || Math.abs(joy.getTwist()) > .4) && joy.getPOV() == -1) {
			frontLeft.drive(wa0, ws0 * throttle);
			backLeft.drive(wa1, ws1 * throttle);
			frontRight.drive(wa2, ws2 * throttle);
			backRight.drive(wa3, ws3 * throttle);
		} else if (joy.getPOV() == 0) {
			this.driveAll(0, throttle);
		} else if (joy.getPOV() == 180) {
			this.driveAll(180, throttle);
		} else if (joy.getPOV() == 90) {
			this.driveAll(90, throttle);
		} else if (joy.getPOV() == 270) {
			this.driveAll(-90, throttle);
		} else {
			frontLeft.disableBoth();
			backLeft.disableBoth();
			frontRight.disableBoth();
			backRight.disableBoth();
		}
	}

	public void turnToDegrees(double angle) {
		frontLeft.turn(39.8);
		backLeft.turn(-39.8);
		frontRight.turn(140.2);
		backRight.turn(-140.2);
		double totalInches = Math.IEEEremainder(angle - navx.getAngle(), 360) / 360.0 * Math.PI * this.r;
		double wheelRevolutions = totalInches / Math.PI / Constants.wheelDiameter;
		double totalEncoderTicks = wheelRevolutions / Constants.driveWheelRatio * 4096;
		frontLeft.driveDistanceMotionMagic(totalEncoderTicks, angle);
		backLeft.driveDistanceMotionMagic(totalEncoderTicks, angle);
		frontRight.driveDistanceMotionMagic(totalEncoderTicks, angle);
		backRight.driveDistanceMotionMagic(totalEncoderTicks, angle);

	}

	public void driveDistanceMotionMagic(double ft, double inches, double angle) {
		double totalInches = (ft * 12) + inches;
		double wheelRevolutions = totalInches / Math.PI / Constants.wheelDiameter;
		double totalEncoderTicks = wheelRevolutions / Constants.driveWheelRatio * 4096;
		frontLeft.turn(angle);
		backLeft.turn(angle);
		frontRight.turn(angle);
		backRight.turn(angle);
		frontLeft.driveDistanceMotionMagic(totalEncoderTicks, angle);
		backLeft.driveDistanceMotionMagic(totalEncoderTicks, angle);
		frontRight.driveDistanceMotionMagic(totalEncoderTicks, angle);
		backRight.driveDistanceMotionMagic(totalEncoderTicks, angle);
	}

	public double motionMagicError() {
		return (frontLeft.drive.getClosedLoopError(0) + backLeft.drive.getClosedLoopError(0)
				+ frontRight.drive.getClosedLoopError(0) + backRight.drive.getClosedLoopError(0)) / 4;
	}

	public void driveDistanceNavX(double ft, double inches, double degree, double breakTimeSeconds,
			double defaultSpeed) {
		t.reset();
		double totalInches = (ft * 12) + inches;
		double wheelRevolutions = totalInches / Math.PI / Constants.wheelDiameter;
		double totalEncoderTicks = wheelRevolutions / Constants.driveWheelRatio * 4096;
		double encoderDistance = totalEncoderTicks;
		final double DEFAULT_SPEED = defaultSpeed;
		double speed = 0;

		frontLeft.resetDriveEncPos();
		backLeft.resetDriveEncPos();
		frontRight.resetDriveEncPos();
		backRight.resetDriveEncPos();
		t.start();
		while ((Math.abs(frontLeft.drive.getSelectedSensorPosition(0)) < totalEncoderTicks - (totalEncoderTicks * .05)
				|| Math.abs(frontRight.drive.getSelectedSensorPosition(0)) < totalEncoderTicks
						- (totalEncoderTicks * .05))
				&& ((breakTimeSeconds != 0) ? t.get() < breakTimeSeconds : true)) {
			speed = DEFAULT_SPEED * (Math.abs(frontLeft.getDriveEncPos() / encoderDistance));
			if (speed < .3) {
				speed = .3;
			}
			if (navx.getAngle() > degree + 1.5) {
				// if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance)
				frontLeft.drive(degree, speed * .95);

				// if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance)
				backLeft.drive(degree, speed * .95);

				// if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance)
				frontRight.drive(degree, speed * 1.05);

				// if (Math.abs(backRight.getDriveEncPos()) < encoderDistance)
				backRight.drive(degree, speed * 1.05);

			} else if (navx.getAngle() < degree - 1.5) {
				// if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance)
				frontLeft.drive(degree, speed * 1.05);

				// if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance)
				backLeft.drive(degree, speed * 1.05);
				// if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance)
				frontRight.drive(degree, speed * .95);

				// if (Math.abs(backRight.getDriveEncPos()) < encoderDistance)
				backRight.drive(degree, speed * .95);

			} else {
				// if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance)
				frontLeft.drive(degree, speed);
				// if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance)
				backLeft.drive(degree, speed);

				// if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance)
				frontRight.drive(degree, speed);

				// if (Math.abs(backRight.getDriveEncPos()) < encoderDistance)
				backRight.drive(degree, speed);

			}
		}
		frontLeft.drive(0, 0);
		backLeft.drive(0, 0);
		frontRight.drive(0, 0);
		backRight.drive(0, 0);

	}

	public void driveDistanceNavX(double ft, double inches, double degree, double breakTimeSeconds) {
		t.reset();
		double totalInches = (ft * 12) + inches;
		double wheelRevolutions = totalInches / Math.PI / Constants.wheelDiameter;
		double totalEncoderTicks = wheelRevolutions / Constants.driveWheelRatio * 4096;
		double encoderDistance = totalEncoderTicks;
		final double DEFAULT_SPEED = .8;
		double speed = 0;

		frontLeft.resetDriveEncPos();
		backLeft.resetDriveEncPos();
		frontRight.resetDriveEncPos();
		backRight.resetDriveEncPos();
		t.start();
		while ((Math.abs(frontLeft.drive.getSelectedSensorPosition(0)) < totalEncoderTicks - (totalEncoderTicks * .05)
				|| Math.abs(frontRight.drive.getSelectedSensorPosition(0)) < totalEncoderTicks
						- (totalEncoderTicks * .05))
				&& ((breakTimeSeconds != 0) ? t.get() < breakTimeSeconds : true)) {
			speed = /* DEFAULT_SPEED * (Math.abs(frontLeft.getDriveEncPos() / encoderDistance)) */.8;
			if (speed < .3) {
				speed = .3;
			}
			if (navx.getAngle() > degree + 1.5) {
				// if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance)
				frontLeft.drive(degree, speed * .95);

				// if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance)
				backLeft.drive(degree, speed * .95);

				// if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance)
				frontRight.drive(degree, speed * 1.05);

				// if (Math.abs(backRight.getDriveEncPos()) < encoderDistance)
				backRight.drive(degree, speed * 1.05);

			} else if (navx.getAngle() < degree - 1.5) {
				// if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance)
				frontLeft.drive(degree, speed * 1.05);

				// if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance)
				backLeft.drive(degree, speed * 1.05);
				// if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance)
				frontRight.drive(degree, speed * .95);

				// if (Math.abs(backRight.getDriveEncPos()) < encoderDistance)
				backRight.drive(degree, speed * .95);

			} else {
				// if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance)
				frontLeft.drive(degree, speed);
				// if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance)
				backLeft.drive(degree, speed);

				// if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance)
				frontRight.drive(degree, speed);

				// if (Math.abs(backRight.getDriveEncPos()) < encoderDistance)
				backRight.drive(degree, speed);

			}
		}
		frontLeft.drive(0, 0);
		backLeft.drive(0, 0);
		frontRight.drive(0, 0);
		backRight.drive(0, 0);

	}

	private double circumEq(double feet, double inches) {
		return (((feet * 12.0) + inches) / (Constants.wheelDiameter * Math.PI)) * 3.2;
	}

	public void calculations() {
		throttle = convertedThrottle();
		fwd = -joy.getY();
		str = joy.getX();
		rcw = joy.getTwist();
		if (!joy.getTrigger()) {
			double temp = fwd * Math.cos(navx.getAngle() * (Math.PI / 180))
					+ str * Math.sin(navx.getAngle() * (Math.PI / 180));
			str = -fwd * Math.sin(navx.getAngle() * (Math.PI / 180))
					+ str * Math.cos(navx.getAngle() * (Math.PI / 180));
			fwd = temp;
		}
		a = str - rcw * (l / r);
		b = str + rcw * (l / r);
		c = fwd - rcw * (w / r);
		d = fwd + rcw * (w / r);
		ws0 = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2)); // front left
		wa0 = Math.atan2(b, d) * 180 / Math.PI;// front left

		ws1 = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2)); // rear left
		wa1 = Math.atan2(a, d) * 180 / Math.PI;// rear left

		ws2 = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));// front right
		wa2 = Math.atan2(b, c) * 180 / Math.PI;// front right

		ws3 = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2)); // rear right
		wa3 = Math.atan2(a, c) * 180 / Math.PI; // rear right
		max = Math.max(Math.max(ws0, ws1), Math.max(ws2, ws3));
		if (max > 1) {
			ws0 /= max;
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
		}

	}

	private double convertedThrottle() {
		return joy.getThrottle() / -2 + .5;
	}

	@Deprecated
	public void resetEncoders() {
		frontLeft.resetSteerEncPos();
		backLeft.resetSteerEncPos();
		frontRight.resetSteerEncPos();
		backRight.resetSteerEncPos();
	}

	public void print() {
		SmartDashboard.putNumber("FrontLeft", frontLeft.getSteerEncPos());
		SmartDashboard.putNumber("BackLeft", backLeft.getSteerEncPos());
		SmartDashboard.putNumber("FrontRight", frontRight.getSteerEncPos());
		SmartDashboard.putNumber("BackRight", backRight.getSteerEncPos());
		SmartDashboard.putNumber("NAVX", navx.getAngle());
		// SmartDashboard.putNumber("Motion Magic Error", this.motionMagicError());
	}

	public void printAbsPos() {
		SmartDashboard.putNumber("ABSOLUTE POSITION", frontLeft.steer.getSensorCollection().getPulseWidthPosition());
	}

	public void init() {
		frontLeft.init();
		backLeft.init();
		frontRight.init();
		backRight.init();
	}

	public void driveAll(double angle, double speed) {
		frontLeft.drive(angle, speed);
		backLeft.drive(angle, speed);
		frontRight.drive(angle, speed);
		backRight.drive(angle, speed);
		// frontLeft.drive(speed);
		// backLeft.drive(speed);
		// frontRight.drive(speed);
		// backRight.drive(speed);

	}

	public void driveAll() {
		frontLeft.drive(wa0, ws0 * throttle);
		backLeft.drive(wa1, ws1 * throttle);
		frontRight.drive(wa2, ws2 * throttle);
		backRight.drive(wa3, ws3 * throttle);
		// frontLeft.drive(ws0 * throttle);
		// backLeft.drive(ws1 * throttle);
		// frontRight.drive(ws2 * throttle);
		// backRight.drive(ws3 * throttle);
	}

	public double getAngle() {
		return navx.getAngle();
	}

	public void disableAll() {
		frontLeft.disableBoth();
		backLeft.disableBoth();
		frontRight.disableBoth();
		backRight.disableBoth();
		// TODO Auto-generated method stub

	}
}