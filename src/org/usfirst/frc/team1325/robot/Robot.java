package org.usfirst.frc.team1325.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;

import java.io.*;
import java.util.*;

public class Robot extends IterativeRobot {
	private static final TalonSRX lfM = new TalonSRX(1);//Has mag encoder plugged in Positive = For
	private static final TalonSRX lrM = new TalonSRX(3);
	private static final TalonSRX rfM = new TalonSRX(15);//Has mag encoder plugged in Negative = For
	private static final TalonSRX rrM = new TalonSRX(12);
	private static final ADXRS453Gyro gyro = new ADXRS453Gyro();
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
		lfM.set(ControlMode.PercentOutput, 0.0);
		lfM.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		lfM.setSensorPhase(true);
		lfM.setNeutralMode(NeutralMode.Brake);
		lfM.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
		lfM.configContinuousCurrentLimit(40, 10);
		lfM.configPeakCurrentLimit(0, 10);
		lfM.configPeakCurrentDuration(0, 10);
		lfM.enableCurrentLimit(true);
		rfM.set(ControlMode.PercentOutput, 0.0);
		rfM.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rfM.setSensorPhase(true);
		rfM.setNeutralMode(NeutralMode.Brake);
		rfM.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
		rfM.configContinuousCurrentLimit(40, 10);
		rfM.configPeakCurrentLimit(0, 10);
		rfM.configPeakCurrentDuration(0, 10);
		rfM.enableCurrentLimit(true);
		lrM.set(ControlMode.PercentOutput, 0.0);
		lrM.setNeutralMode(NeutralMode.Brake);
		lrM.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
		lrM.configContinuousCurrentLimit(40, 10);
		lrM.configPeakCurrentLimit(0, 10);
		lrM.configPeakCurrentDuration(0, 10);
		lrM.enableCurrentLimit(true);
		rrM.set(ControlMode.PercentOutput, 0.0);
		rrM.setNeutralMode(NeutralMode.Brake);
		rrM.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
		rrM.configContinuousCurrentLimit(40, 10);
		rrM.configPeakCurrentLimit(0, 10);
		rrM.configPeakCurrentDuration(0, 10);
		rrM.enableCurrentLimit(true);
	}

	public void teleopInit() {
		logThread = new Thread(() -> {
			boolean secondSide = false;
			double start, start2, startTime = start = start2 = time = System.nanoTime();
			while(!Thread.interrupted()) {
				delta = (System.nanoTime() - startTime) / 1000000000.0;//While loop period
				if(((System.nanoTime() - start) / 1000000000.0) < totalTime / 2.0) {//Which motor
					if(delta >= samplePeriod) {
						avgdT += delta;
						counter++;
						startTime = System.nanoTime();
						//y = sin(0.4x^2)
						double val = Math.sin(0.4 * Math.pow((startTime - start2) / 1000000000.0, 2));
						Robot.rfM.set(ControlMode.PercentOutput, val);
						Robot.rrM.set(ControlMode.PercentOutput, val);
						logs[counter].lfM = lfM.getMotorOutputPercent();
						logs[counter].lrM = lrM.getMotorOutputPercent();
						logs[counter].rfM = rfM.getMotorOutputPercent();
						logs[counter].rrM = rrM.getMotorOutputPercent();
						logs[counter].leftCmd = 0.0;
						logs[counter].rightCmd = val;
						logs[counter].totalLeftCurr = lfM.getOutputCurrent() + lrM.getOutputCurrent();
						logs[counter].totalRightCurr = rfM.getOutputCurrent() + rrM.getOutputCurrent();
						logs[counter].leftVelNativeUnits = lfM.getSelectedSensorVelocity(0);
						logs[counter].rightVelNativeUnits = rfM.getSelectedSensorVelocity(0);
						logs[counter].gyroRate = gyro.getRate();
						logs[counter].time = (System.nanoTime() - time) / 1000000000.0;
					}
				} else if((((System.nanoTime() - start) / 1000000000.0) >= totalTime / 2.0) && (((System.nanoTime() - start) / 1000000000.0)
						<= totalTime)) {
					if(!secondSide) {
						secondSide = true;
						Robot.rfM.set(ControlMode.PercentOutput, 0.0);
						Robot.rrM.set(ControlMode.PercentOutput, 0.0);
						start2 = System.nanoTime();
					}
					if(delta >= samplePeriod) {
						avgdT += delta;
						counter++;
						startTime = System.nanoTime();
						//y = sin(0.4x^2)
						double val = Math.sin(0.4 * Math.pow((startTime - start2) / 1000000000.0, 2));
						Robot.lfM.set(ControlMode.PercentOutput, val);
						Robot.lrM.set(ControlMode.PercentOutput, val);
						logs[counter].lfM = lfM.getMotorOutputPercent();
						logs[counter].lrM = lrM.getMotorOutputPercent();
						logs[counter].rfM = rfM.getMotorOutputPercent();
						logs[counter].rrM = rrM.getMotorOutputPercent();
						logs[counter].leftCmd = val;
						logs[counter].rightCmd = 0.0;
						logs[counter].totalLeftCurr = lfM.getOutputCurrent() + lrM.getOutputCurrent();
						logs[counter].totalRightCurr = rfM.getOutputCurrent() + rrM.getOutputCurrent();
						logs[counter].leftVelNativeUnits = lfM.getSelectedSensorVelocity(0);
						logs[counter].rightVelNativeUnits = rfM.getSelectedSensorVelocity(0);
						logs[counter].gyroRate = gyro.getRate();
						logs[counter].time = (System.nanoTime() - time) / 1000000000.0;
					}
				} else {
					time = System.nanoTime();
					Robot.lfM.set(ControlMode.PercentOutput, 0.0);
					Robot.lrM.set(ControlMode.PercentOutput, 0.0);
					Robot.rfM.set(ControlMode.PercentOutput, 0.0);
					Robot.rrM.set(ControlMode.PercentOutput, 0.0);
					return;
				}
			}
		});
		logThread.start();
	}

	public void disabledInit() {
		Robot.lfM.set(ControlMode.PercentOutput, 0.0);
		Robot.lrM.set(ControlMode.PercentOutput, 0.0);
		Robot.rfM.set(ControlMode.PercentOutput, 0.0);
		Robot.rrM.set(ControlMode.PercentOutput, 0.0);
		logThread.interrupt();

		try {
			Thread.sleep(1000);
		} catch(InterruptedException e) {
			e.printStackTrace();
		}

		if(logThread.isInterrupted() || (!logThread.isAlive() && !Double.isNaN(avgdT / counter))) {
			System.out.println("pls work");
			File f = new File("/home/lvuser/logs");
			if(!f.exists()) f.mkdir();
			try {
				BufferedWriter bw = new BufferedWriter(new FileWriter(String.format("/home/lvuser/logs/%tB%te%tY-%tl%tM%tS%tp.csv",
						c, c, c, c, c, c, c)));
				bw.write("lfM, lrM, rfM, rrM, leftCmd, rightCmd, totalLeftCurr, totalRightCurr, leftVelNativeUnits, rightVelNativeUnits, gyroRate, time");
				bw.newLine();
				for(Data log : logs) {
					bw.write(log.lfM + "," + log.lrM + "," + log.rfM + "," + log.rrM + "," + log.leftCmd + "," + log.rightCmd + "," + log.totalLeftCurr + ","
							+ log.totalRightCurr + "," + log.leftVelNativeUnits + "," + log.rightVelNativeUnits + "," + log.gyroRate + "," + log.time);
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
		Double lfM, lrM, rfM, rrM, leftCmd, rightCmd, totalLeftCurr, totalRightCurr, time, gyroRate;
		int leftVelNativeUnits, rightVelNativeUnits;

		Data() {
			this.lfM = 0.0;
			this.lrM = 0.0;
			this.rfM = 0.0;
			this.rrM = 0.0;
			this.leftCmd = 0.0;
			this.rightCmd = 0.0;
			this.totalLeftCurr = 0.0;
			this.totalRightCurr = 0.0;
			this.leftVelNativeUnits = 0;
			this.rightVelNativeUnits = 0;
			this.gyroRate = 0.0;
			this.time = 0.0;
		}
	}
}