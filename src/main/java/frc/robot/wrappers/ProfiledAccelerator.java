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
        System.out.print("Delta: ");
        System.out.println(delta);
        if (Math.signum(delta) == Math.signum(newValue)) {
            System.out.println("Calculating");
            // If delta is negative and target is negative or if delta is positive and target is positive
            delta = Math.min(m_maxIncreasePerTick, Math.abs(delta)); // Limit the change
            System.out.print("Limited Delta: ");
            System.out.println(delta);
            delta = delta * Math.signum(newValue); // Restore sign
            System.out.print("Limited & Signed Delta: ");
            System.out.println(delta);
            double calculatedValue = m_oldValue + delta; // Calculate new change
            m_oldValue = calculatedValue;
            System.out.print("Calculated: ");
            System.out.println(calculatedValue);
            System.out.print("New Old Value: ");
            System.out.println(m_oldValue);
            return calculatedValue;
        }
        System.out.println("Skipped");
        m_oldValue = newValue;
        return newValue;
    }
}
