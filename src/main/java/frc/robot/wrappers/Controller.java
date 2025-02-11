// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Controller extends CommandXboxController {
    /** Create a new Controller wrapper */
    public Controller(Integer port) {
        super(port);
    }
    /**
     * @param name the name of the button. One of a, b, x, y, start, back, up, down, left, right, lb, rb, stick_left, stick_right
     * @return the output of the button named 'name'
     */
    public Trigger getByName(String name) {
        if (name.equals("a")) {
            return this.a();
        }
        if (name.equals("b")) {
            return this.b();
        }
        if (name.equals("x")) {
            return this.x();
        }
        if (name.equals("y")) {
            return this.y();
        }
        if (name.equals("start")) {
            return this.start();
        }
        if (name.equals("back")) {
            return this.back();
        }
        if (name.equals("up")) {
            return this.povUp();
        }
        if (name.equals("down")) {
            return this.povDown();
        }
        if (name.equals("left")) {
            return this.povLeft();
        }
        if (name.equals("right")) {
            return this.povRight();
        }
        if (name.equals("lb")) {
            return this.leftTrigger();
        }
        if (name.equals("rb")) {
            return this.rightTrigger();
        }
        if (name.equals("stick_left")) {
            return this.leftStick();
        }
        if (name.equals("stick_right")) {
            return this.rightStick();
        }
        throw new Error("Unknown Controller Input Name " + name);
    }
}
