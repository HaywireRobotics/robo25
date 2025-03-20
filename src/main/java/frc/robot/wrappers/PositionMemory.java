// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

/** A simple counter that automatically limits itself to a min and a max. */
public class PositionMemory {
    private int m_point = 1;
    private final int m_min;
    private final int m_max;

    public PositionMemory(int min, int max) {
        m_min = min;
        m_max = max;
    }

    public void raise() {
        if (m_point >= m_max) {
            return;
        }
        m_point += 1;
    }

    public void lower() {
        if (m_point <= m_min) {
            return;
        }
        m_point -= 1;
    }

    public int get() {
        return m_point;
    }

    public int set(int value) {
        m_point = Math.min(m_max, Math.max(m_min, value));
        return m_point;
    }
}
