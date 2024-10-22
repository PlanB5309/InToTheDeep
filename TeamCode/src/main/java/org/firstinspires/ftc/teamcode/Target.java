package org.firstinspires.ftc.teamcode;

public class Target {
    enum Destination {
        WAYPOINT,
        DESTINATION
    }
    double x;
    double y;
    double h;
    double maxSpeed;
    Destination type;



    public Target(double x, double y, double h, double maxSpeed, Destination type){
        this.x = x;
        this.y = y;
        this.h = h;
        this.maxSpeed = maxSpeed;
        this.type = type;
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
