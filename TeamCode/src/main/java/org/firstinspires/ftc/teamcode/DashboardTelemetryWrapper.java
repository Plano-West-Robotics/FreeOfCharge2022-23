package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kinematics.Distance;
import org.firstinspires.ftc.teamcode.kinematics.FieldVector;
import org.firstinspires.ftc.teamcode.kinematics.Pose;

public class DashboardTelemetryWrapper implements Telemetry {
    private final FtcDashboard dashboard;
    private TelemetryPacket packet;
    private final TelemetryLog log;

    public DashboardTelemetryWrapper(FtcDashboard dashboard) {
        this.dashboard = dashboard;
        this.packet = new TelemetryPacket();
        this.log = this.new TelemetryLog();
    }

    private void rawDrawRobot(FieldVector pos, double yaw) {
        // 2--3    6--7
        // |  |    |  |
        // |  4----5  |
        // |          |
        // |          |
        // 1----------8
        FieldVector[] ps = new FieldVector[]{
                new FieldVector( Distance.ROBOT_WIDTH.neg(),        Distance.ROBOT_LENGTH.neg()  ).div(2),
                new FieldVector( Distance.ROBOT_WIDTH.neg(),        Distance.ROBOT_LENGTH        ).div(2),
                new FieldVector( Distance.ROBOT_WIDTH.neg().div(2), Distance.ROBOT_LENGTH        ).div(2),
                new FieldVector( Distance.ROBOT_WIDTH.neg().div(2), Distance.ROBOT_LENGTH.div(3) ).div(2),
                new FieldVector( Distance.ROBOT_WIDTH.div(2),       Distance.ROBOT_LENGTH.div(3) ).div(2),
                new FieldVector( Distance.ROBOT_WIDTH.div(2),       Distance.ROBOT_LENGTH        ).div(2),
                new FieldVector( Distance.ROBOT_WIDTH,              Distance.ROBOT_LENGTH        ).div(2),
                new FieldVector( Distance.ROBOT_WIDTH,              Distance.ROBOT_LENGTH.neg()  ).div(2)
        };

        double[] xs = new double[ps.length];
        double[] ys = new double[ps.length];
        for (int i = 0; i < ps.length; i ++) {
            FieldVector p = ps[i].rot(yaw).add(pos);
            xs[i] = p.x.valInInches();
            ys[i] = p.y.valInInches();
        }

        Canvas canvas = this.packet.fieldOverlay();
        canvas.fillPolygon(xs, ys);
    }

    public void drawRobot(FieldVector pos, double yaw) {
        Canvas canvas = this.packet.fieldOverlay();
        canvas.setStrokeWidth(1);
        canvas.setFill("gray");
        canvas.setStroke("black");

        this.rawDrawRobot(pos, yaw);
    }

    public void drawTarget(Pose target, Pose current) {
        Canvas canvas = this.packet.fieldOverlay();
        canvas.setStrokeWidth(1);
        canvas.setFill("green");
        canvas.setStroke("green");

        canvas.fillCircle(target.pos.x.valInInches(), target.pos.y.valInInches(), 2);
        canvas.strokeLine(
                current.pos.x.valInInches(),
                current.pos.y.valInInches(),
                target.pos.x.valInInches(),
                target.pos.y.valInInches()
        );

        canvas.setFill("transparent");
        this.rawDrawRobot(target.pos, target.yaw);
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return this.addData(caption, String.format(format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        this.packet.put(caption, value);
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return this.addData(caption, valueProducer.value());
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return this.addData(caption, format, valueProducer.value());
    }

    @Override
    public boolean removeItem(Item item) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void clear() {
        this.dashboard.clearTelemetry();
        this.packet = new TelemetryPacket();
    }

    @Override
    public void clearAll() {
        this.clear();
    }

    @Override
    public Object addAction(Runnable action) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean removeAction(Object token) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void speak(String text) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean update() {
        this.dashboard.sendTelemetryPacket(packet);
        this.packet = new TelemetryPacket();
        return true;
    }

    @Override
    public Line addLine() {
        return null;
    }

    @Override
    public Line addLine(String lineCaption) {
        this.packet.addLine(lineCaption);
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getMsTransmissionInterval() {
        return this.dashboard.getTelemetryTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        this.dashboard.setTelemetryTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return null;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {

    }

    @Override
    public String getCaptionValueSeparator() {
        return null;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {

    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {

    }

    @Override
    public Log log() {
        return log;
    }

    private class TelemetryLog implements Log {
        @Override
        public int getCapacity() {
            return 0;
        }

        @Override
        public void setCapacity(int capacity) {

        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return DisplayOrder.OLDEST_FIRST;
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {

        }

        @Override
        public void add(String entry) {
            packet.addLine(entry);
        }

        @Override
        public void add(String format, Object... args) {
            this.add(String.format(format, args));
        }

        @Override
        public void clear() {
            packet.clearLines();
        }
    }
}
