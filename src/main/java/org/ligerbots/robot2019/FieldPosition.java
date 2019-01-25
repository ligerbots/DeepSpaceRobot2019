package org.ligerbots.robot2019;

public class FieldPosition {

    double x;
    double y;

    public FieldPosition(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public FieldPosition add(FieldPosition other) {
      return new FieldPosition(x + other.x, y + other.y);
    }

    public FieldPosition add(double x, double y) {
      return new FieldPosition(this.x + x, this.y + y);
    }

    public FieldPosition multiply(double xFactor, double yFactor) {
      return new FieldPosition(x * xFactor, y * yFactor);
    }

    public FieldPosition multiply(double mxy) {
      return multiply(mxy, mxy);
    }

    public double angleTo(FieldPosition other) {
      return Math.toDegrees(Math.atan2(other.y - y, other.x - x));
    }

    public double distanceTo(FieldPosition other) {
      double dx = other.x - x;
      double dy = other.y - y;
      return Math.sqrt(dx * dx + dy * dy);
    }

    public double getX () {
      return x;
    }

    public double getY () {
      return y;
    }
}

