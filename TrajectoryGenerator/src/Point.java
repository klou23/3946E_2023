/**
 * File: Point.java
 * Author: Kevin Lou
 * Created On: Dec 23 2022
 * Last Updated: Dec 26 2022
 *
 * Copyright (c) 2022 3946E
 *
 * Summary of file:
 *  - A point representing a position in (x,y) coordinate space, specified in
 *      double floating point precision.
 */

import java.util.Objects;

public class Point {

    double x;
    double y;

    /**
     * Constructs a point at the origin (0,0)
     */
    public Point(){
        this(0,0);
    }

    /**
     * Constructs a point at the specified (x,y) coordinate
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Calculates and returns the distance from this point to another point
     */
    double distTo(Point a){
        double dx = this.x - a.x;
        double dy = this.y - a.y;
        return Math.sqrt(dx*dx + dy*dy);
    }

    /**
     * Calculates and returns the curvature of the path at this point given
     * the point before and after this point
     * @param a the point before this point in the trajectory
     * @param b the point after this point in the trajectory
     */
    double findCurvature(Point a, Point b){
        double x1 = a.x;
        double y1 = a.y;
        double x2 = this.x;
        double y2 = this.y;
        double x3 = b.x;
        double y3 = b.y;

        double A = x1*(y2-y3)-y1*(x2-x3)+x2*y3-x3*y2;
        double B = (x1*x1+y1*y1)*(y3-y2)+(x2*x2+y2*y2)*(y1-y3)+
                (x3*x3+y3*y3)*(y2-y1);
        double C = (x1*x1+y1*y1)*(x2-x3)+(x2*x2+y2*y2)*(x3-x1)+
                (x3*x3+y3*y3)*(x1-x2);
        double D = (x1*x1+y1*y1)*(x3*y2-x2*y3)+(x2*x2+y2*y2)*(x1*y3-x3*y1)+
                (x3*x3+y3*y3)*(x2*y1-x1*y2);

        if(A < 0.0001) return 0;
        double r = Math.sqrt((B*B+C*C-4*A*D)/(4*A*A));
        return 1.0/r;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Point point = (Point) o;
        return Double.compare(point.x, x) == 0 && Double.compare(point.y, y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }
}