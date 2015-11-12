import model.Game;
import model.World;

import java.awt.*;

import static java.lang.StrictMath.*;

public final class LocalTestRendererListener {
    private static final LocalServer server = LocalServer.INSTANCE;

    static {
        server.run();
    }

    private Graphics graphics;
    private World world;
    private Game game;

    private int canvasWidth;
    private int canvasHeight;

    private double left;
    private double top;
    private double width;
    private double height;

    public void beforeDrawScene(Graphics graphics, World world, Game game, int canvasWidth, int canvasHeight,
                                double left, double top, double width, double height) {
        ConstDump.run(game);

        while (true) {
            String message = server.messages.poll();
            if (message == null) break;
            handleMessage(graphics, message);
        }

        updateFields(graphics, world, game, canvasWidth, canvasHeight, left, top, width, height);
    }

    private void handleMessage(Graphics graphics, String message) {
        String[] command = message.split(" ");
        if (command.length == 0) return;

        double[] args = new double[command.length - 1];
        for (int i = 0; i < command.length - 1; i++) {
            try {
                args[i] = Double.valueOf(command[i + 1]);
            } catch (NumberFormatException e) {
                return;
            }
        }

        switch (command[0]) {
            case "rect":
                drawRect(args[0], args[1], args[2], args[3]);
                break;
            case "line":
                drawLine(args[0], args[1], args[2], args[3]);
                break;
            default:
                log("unknown message: " + message);
                break;
        }
    }

    public void afterDrawScene(Graphics graphics, World world, Game game, int canvasWidth, int canvasHeight,
                               double left, double top, double width, double height) {
        updateFields(graphics, world, game, canvasWidth, canvasHeight, left, top, width, height);
    }

    private void updateFields(Graphics graphics, World world, Game game, int canvasWidth, int canvasHeight,
                              double left, double top, double width, double height) {
        this.graphics = graphics;
        this.world = world;
        this.game = game;

        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;

        this.left = left;
        this.top = top;
        this.width = width;
        this.height = height;
    }

    private void drawLine(double x1, double y1, double x2, double y2) {
        Point2I lineBegin = toCanvasPosition(x1, y1);
        Point2I lineEnd = toCanvasPosition(x2, y2);

        graphics.drawLine(lineBegin.x, lineBegin.y, lineEnd.x, lineEnd.y);
    }

    private void fillCircle(double centerX, double centerY, double radius) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.fillOval(topLeft.x, topLeft.y, size.x, size.y);
    }

    private void drawCircle(double centerX, double centerY, double radius) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.drawOval(topLeft.x, topLeft.y, size.x, size.y);
    }

    private void fillArc(double centerX, double centerY, double radius, int startAngle, int arcAngle) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.fillArc(topLeft.x, topLeft.y, size.x, size.y, startAngle, arcAngle);
    }

    private void drawArc(double centerX, double centerY, double radius, int startAngle, int arcAngle) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.drawArc(topLeft.x, topLeft.y, size.x, size.y, startAngle, arcAngle);
    }

    private void fillRect(double left, double top, double width, double height) {
        Point2I topLeft = toCanvasPosition(left, top);
        Point2I size = toCanvasOffset(width, height);

        graphics.fillRect(topLeft.x, topLeft.y, size.x, size.y);
    }

    private void drawRect(double left, double top, double width, double height) {
        Point2I topLeft = toCanvasPosition(left, top);
        Point2I size = toCanvasOffset(width, height);

        graphics.drawRect(topLeft.x, topLeft.y, size.x, size.y);
    }

    private void drawPolygon(Point2D... points) {
        int pointCount = points.length;

        for (int pointIndex = 1; pointIndex < pointCount; ++pointIndex) {
            Point2D pointA = points[pointIndex];
            Point2D pointB = points[pointIndex - 1];
            drawLine(pointA.x, pointA.y, pointB.x, pointB.y);
        }

        Point2D pointA = points[0];
        Point2D pointB = points[pointCount - 1];
        drawLine(pointA.x, pointA.y, pointB.x, pointB.y);
    }

    private Point2I toCanvasOffset(double x, double y) {
        return new Point2I(x * canvasWidth / width, y * canvasHeight / height);
    }

    private Point2I toCanvasPosition(double x, double y) {
        return new Point2I((x - left) * canvasWidth / width, (y - top) * canvasHeight / height);
    }

    private static final class Point2I {
        public final int x;
        public final int y;

        public Point2I(double x, double y) {
            this.x = toInt(round(x));
            this.y = toInt(round(y));
        }

        public Point2I(int x, int y) {
            this.x = x;
            this.y = y;
        }

        private static int toInt(double value) {
            @SuppressWarnings("NumericCastThatLosesPrecision") int intValue = (int) value;
            if (abs((double) intValue - value) < 1.0D) {
                return intValue;
            }
            throw new IllegalArgumentException("Can't convert double " + value + " to int.");
        }
    }

    private static final class Point2D {
        public final double x;
        public final double y;

        public Point2D(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private static void warn(String message) {
        System.err.println("renderer: " + message);
    }

    private static void log(String message) {
        // System.err.println("renderer: " + message);
    }
}
