package org.usfirst.frc.team1325.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;

import java.io.*;
import java.util.*;

public class Robot extends IterativeRobot {
	private static final TalonSRX _driveLeftFront = new TalonSRX(1);//Has mag encoder plugged in Positive = For
	private static final TalonSRX _driveLeftRear = new TalonSRX(3);
	private static final TalonSRX _driveRightFront = new TalonSRX(15);//Has mag encoder plugged in Negative = For
	private static final TalonSRX _driveRightRear = new TalonSRX(12);
	private double delta, avgdT, time = avgdT = delta = 0.0, totalTime = 60.0, samplePeriod = 0.015;
	private Data[] logs = new Data[(int) (totalTime / samplePeriod)];
	private Thread logThread = new Thread();
	private int counter = 0;
	private Calendar c = Calendar.getInstance();

	{
		Arrays.setAll(logs, i -> new Data());
	}

	@Override
	public void robotInit() {
		_driveLeftFront.set(ControlMode.PercentOutput, 0.0);
		_driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		_driveLeftFront.setSensorPhase(true);
		_driveLeftFront.setNeutralMode(NeutralMode.Brake);
		_driveLeftFront.configContinuousCurrentLimit(40, 10);
		_driveLeftFront.configPeakCurrentLimit(0, 10);
		_driveLeftFront.configPeakCurrentDuration(0, 10);
		_driveLeftFront.enableCurrentLimit(true);
		_driveRightFront.set(ControlMode.PercentOutput, 0.0);
		_driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		_driveRightFront.setSensorPhase(true);
		_driveRightFront.setNeutralMode(NeutralMode.Brake);
		_driveRightFront.configContinuousCurrentLimit(40, 10);
		_driveRightFront.configPeakCurrentLimit(0, 10);
		_driveRightFront.configPeakCurrentDuration(0, 10);
		_driveRightFront.enableCurrentLimit(true);
		_driveLeftRear.set(ControlMode.PercentOutput, 0.0);
		_driveLeftRear.setNeutralMode(NeutralMode.Brake);
		_driveLeftRear.configContinuousCurrentLimit(40, 10);
		_driveLeftRear.configPeakCurrentLimit(0, 10);
		_driveLeftRear.configPeakCurrentDuration(0, 10);
		_driveLeftRear.enableCurrentLimit(true);
		_driveRightRear.set(ControlMode.PercentOutput, 0.0);
		_driveRightRear.setNeutralMode(NeutralMode.Brake);
		_driveRightRear.configContinuousCurrentLimit(40, 10);
		_driveRightRear.configPeakCurrentLimit(0, 10);
		_driveRightRear.configPeakCurrentDuration(0, 10);
		_driveRightRear.enableCurrentLimit(true);
	}

	public void teleopInit() {
		logThread = new Thread(() -> {
			boolean secondSide = true;
			double start, start2, startTime = start = start2 = time = System.nanoTime();
			while(!Thread.interrupted()) {
				delta = (System.nanoTime() - startTime) / 1000000000.0;//While loop period
				if(((System.nanoTime() - start) / 1000000000.0) < totalTime / 2.0) {//Which side
					if(delta >= samplePeriod) {
						avgdT += delta;
						counter++;
						startTime = System.nanoTime();
						//y = -sin(0.4x^2)
						Robot._driveRightFront.set(ControlMode.PercentOutput, -1 * Math.sin(0.4 * Math.pow((startTime - start2) /
								1000000000.0, 2)));
						Robot._driveRightRear.set(ControlMode.PercentOutput, -1 * Math.sin(0.4 * Math.pow((startTime - start2) /
								1000000000.0, 2)));
						logs[counter].leftFront = _driveLeftFront.getMotorOutputPercent();
						logs[counter].leftRear = _driveLeftRear.getMotorOutputPercent();
						logs[counter].rightFront = _driveRightFront.getMotorOutputPercent();
						logs[counter].rightRear = _driveRightRear.getMotorOutputPercent();
						logs[counter].leftVelNativeUnits = _driveLeftFront.getSelectedSensorVelocity(0);
						logs[counter].rightVelNativeUnits = _driveRightFront.getSelectedSensorVelocity(0);
						logs[counter].time = (System.nanoTime() - time) / 1000000000.0;
					}
				} else if((((System.nanoTime() - start) / 1000000000.0) >= totalTime / 2.0) && (((System.nanoTime() - start) / 1000000000.0)
						<= totalTime)) {
					if(secondSide) {
						secondSide = false;
						Robot._driveRightFront.set(ControlMode.PercentOutput, 0.0);
						Robot._driveRightRear.set(ControlMode.PercentOutput, 0.0);
						start2 = System.nanoTime();
					}
					if(delta >= samplePeriod) {
						avgdT += delta;
						counter++;
						startTime = System.nanoTime();
						Robot._driveLeftFront.set(ControlMode.PercentOutput, Math.sin(0.4 * Math.pow((startTime - start2) / 1000000000.0, 2)));
						Robot._driveLeftRear.set(ControlMode.PercentOutput, Math.sin(0.4 * Math.pow((startTime - start2) / 1000000000.0, 2)));
						logs[counter].leftFront = _driveLeftFront.getMotorOutputPercent();
						logs[counter].leftRear = _driveLeftRear.getMotorOutputPercent();
						logs[counter].rightFront = _driveRightFront.getMotorOutputPercent();
						logs[counter].rightRear = _driveRightRear.getMotorOutputPercent();
						logs[counter].leftVelNativeUnits = _driveLeftFront.getSelectedSensorVelocity(0);
						logs[counter].rightVelNativeUnits = _driveRightFront.getSelectedSensorVelocity(0);
						logs[counter].time = (System.nanoTime() - time) / 1000000000.0;
					}
				} else {
					time = System.nanoTime();
					Robot._driveLeftFront.set(ControlMode.PercentOutput, 0.0);
					Robot._driveLeftRear.set(ControlMode.PercentOutput, 0.0);
					Robot._driveRightFront.set(ControlMode.PercentOutput, 0.0);
					Robot._driveRightRear.set(ControlMode.PercentOutput, 0.0);
				}
			}
		});
		logThread.start();
	}

	public void disabledInit() {
		Robot._driveLeftFront.set(ControlMode.PercentOutput, 0.0);
		Robot._driveLeftRear.set(ControlMode.PercentOutput, 0.0);
		Robot._driveRightFront.set(ControlMode.PercentOutput, 0.0);
		Robot._driveRightRear.set(ControlMode.PercentOutput, 0.0);
		logThread.interrupt();

		if(logThread.isInterrupted()) {
			System.out.println("pls work");
			File f = new File("/home/lvuser/logs");
			if(!f.exists()) f.mkdir();
			try {
				BufferedWriter bw = new BufferedWriter(new FileWriter(String.format("/home/lvuser/logs/%tB%te%tY-%tl%tM%tS%tp.csv",
						c, c, c, c, c, c, c)));
				bw.write("leftFront, leftRear, rightFront, rightRear, leftVelNativeUnits, rightVelNativeUnits, time");
				bw.newLine();
				for(Data log : logs) {
					bw.write(log.leftFront + "," + log.leftRear + "," + log.rightFront + "," + log.rightRear + "," + log.leftVelNativeUnits
							+ "," + log.rightVelNativeUnits + "," + log.time);
					bw.newLine();
				}
				bw.close();
				System.out.println("DONE " + logs.length + " " + avgdT / counter + " " + (System.nanoTime() - time) / 1000000000);
			} catch(IOException e) {
				e.printStackTrace();
			}
		}
	}

	class Data {
		Double leftFront, leftRear, rightFront, rightRear, time;
		int leftVelNativeUnits, rightVelNativeUnits;

		Data() {
			this.leftFront = 0.0;
			this.leftRear = 0.0;
			this.rightFront = 0.0;
			this.rightRear = 0.0;
			this.leftVelNativeUnits = 0;
			this.rightVelNativeUnits = 0;
			this.time = 0.0;
		}
	}
}