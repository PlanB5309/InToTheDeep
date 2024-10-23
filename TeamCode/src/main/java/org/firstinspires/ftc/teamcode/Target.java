package org.firstinspires.ftc.teamcode;

public class Target {
    double x;
    double y;
    double h;
    double maxSpeed;

    public Target(double x, double y, double h, double maxSpeed){
        this.x = x;
        this.y = y;
        this.h = h;
        this.maxSpeed = maxSpeed;
    }

    public Target(){
        x = 0;
        y = 0;
        h = 0;
        maxSpeed = 0;
    }

    public void set (double x, double y, double h, double maxSpeed){
        this.x = x;
        this.y = y;
        this.h = h;
        this.maxSpeed = maxSpeed;
    }
}
