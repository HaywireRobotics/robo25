// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

/** Limits increase but not decrease. */
public class ProfiledAccelerator {
    private final double m_maxIncreasePerTick;
    private double m_oldValue = 0;

    public ProfiledAccelerator(double maxIncreasePerTick) {
        m_maxIncreasePerTick = maxIncreasePerTick;
    }

    public double calculate(double newValue) {
        double delta = newValue - m_oldValue;
        if (Math.signum(newValue) != Math.signum(m_oldValue)) {
            m_oldValue = 0;
        }
        if (Math.signum(delta) == Math.signum(newValue)) {
            // If delta is negative and target is negative or if delta is positive and target is positive
            delta = Math.min(m_maxIncreasePerTick, Math.abs(delta)); // Limit the change
            delta = delta * Math.signum(newValue); // Restore sign
            double calculatedValue = m_oldValue + delta; // Calculate new change
            m_oldValue = calculatedValue;
            return calculatedValue;
        }
        
        m_oldValue = newValue;
        return newValue;
    }
}
