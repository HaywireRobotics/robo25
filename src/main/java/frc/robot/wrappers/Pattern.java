// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Pattern {
    private final Timer m_timer = new Timer();
    private final double[] m_times;


    /**
     * Creates a VibrationPattern that represents the on and off periods of a vibration.
     * 
     * @param times A list of on and off times. For example, [100, 100, 100, 100]. 100 ms on, 100 ms off, 100 ms on, 100 ms off.
     */
    public Pattern(double ...times) {
        m_times = times;
    }

    public void restart() {
        m_timer.restart();
    }

    public boolean getState() {
        return getStateAtTime((m_timer.get()) * 1000);
    }

    public boolean getStateAtTime(double time) {
        double startTime = 0;
        boolean state = false;
        for (int i = 0; i < m_times.length; i++) {
            double endTime = startTime + m_times[i];
            state = !state;
            if (startTime < time && time <= endTime) {
                return state;
            }
            startTime = endTime;
        }
        return false;
    }
}
