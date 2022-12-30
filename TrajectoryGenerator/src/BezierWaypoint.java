/**
 * File: BezierWaypoint.java
 * Author: Kevin Lou
 * Created On: Dec 23 2022
 * Last Updated: Dec 29 2022
 *
 * Copyright (c) 2022 3946E
 *
 * Summary of file:
 *  - Representation of a waypoint in a bezier path
 */

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

public class BezierWaypoint {

    double x;
    double y;
    double theta;
    double beforeDist;
    double afterDist;

    public BezierWaypoint(double x, double y, double theta, double beforeDist, double afterDist) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.beforeDist = beforeDist;
        this.afterDist = afterDist;
    }

    public Point getPoint(){
        return new Point(x, y);
    }

    public Point getBeforeControlPoint(){
        return new Point(x - beforeDist*Math.cos(theta), y - beforeDist*Math.sin(theta));
    }

    public Point getAfterControlPoint(){
        return new Point(x + afterDist*Math.cos(theta), y + afterDist*Math.sin(theta));
    }

    public void draw(Graphics2D g, double[][] transform){
        int x0 = (int) Math.round(x*transform[0][0] + y*transform[0][1] + transform[0][2]);
        int y0 = (int) Math.round(x*transform[1][0] + y*transform[1][1] + transform[1][2]);

        double controlX = getBeforeControlPoint().x;
        double controlY = getBeforeControlPoint().y;
        int x1 = (int) Math.round(controlX*transform[0][0] + controlY*transform[0][1] + transform[0][2]);
        int y1 = (int) Math.round(controlX*transform[1][0] + controlY*transform[1][1] + transform[1][2]);

        controlX = getAfterControlPoint().x;
        controlY = getAfterControlPoint().y;
        int x2 = (int) Math.round(controlX*transform[0][0] + controlY*transform[0][1] + transform[0][2]);
        int y2 = (int) Math.round(controlX*transform[1][0] + controlY*transform[1][1] + transform[1][2]);

        g.setColor(Color.MAGENTA);
        g.setStroke(new BasicStroke(3));
        g.drawLine(x1, y1, x2, y2);
        g.fillOval(x1-4, y1-4, 8, 8);
        g.fillOval(x2-4, y2-4, 8, 8);

        g.setColor(Color.BLACK);
        g.fillOval(x0-6, y0-6, 12, 12);
    }
}
