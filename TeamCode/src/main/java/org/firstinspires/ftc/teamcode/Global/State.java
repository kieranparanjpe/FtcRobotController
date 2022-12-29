package org.firstinspires.ftc.teamcode.Global;

public abstract class State {

    protected double progress = 0;
    protected double timeStarted = 0;
    public boolean runAsync = false;

    public double Run()
    {
        return 1;
    }
}
