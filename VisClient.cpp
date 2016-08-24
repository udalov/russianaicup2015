#include "VisClient.h"

#include <iostream>
#include <vector>
#include <sstream>

using namespace std;

void writeBytes(CActiveSocket& socket, char *bytes, size_t length) {
    size_t offset = 0;
    size_t sentByteCount;
    while (offset < length && (sentByteCount = socket.Send((uint8 *) bytes, length - offset)) > 0) {
        offset += sentByteCount;
    }
    if (offset != length) {
        cerr << "failed to send: " << offset << " != " << length << endl;
    }
}

void writeString(CActiveSocket& socket, const string& str) {
    auto len = str.length();
    if (len > 65535) {
        cerr << "cannot send very large string (" << len << " chars): " << str << endl;
        return;
    }

    static char *buf = new char[65535];
    buf[0] = static_cast<char>((len >> 8) & 0xFF);
    buf[1] = static_cast<char>(len & 0xFF);
    writeBytes(socket, buf, 2);

    str.copy(buf, len);
    writeBytes(socket, buf, len);
}

void VisClient::send(const string& message) {
    if (!valid) return;
    writeString(socket, message);
}

VisClient::VisClient(int port) {
    if (port == -1) {
        valid = false;
        return;
    }

    socket.Initialize();
    socket.DisableNagleAlgoritm();

    if (!socket.Open((uint8 *) "localhost", (int16) port)) {
        valid = false;
        return;
    }

    valid = true;
    // cerr << "vis client socket opened on port " << port << endl;

    send("nop");
}

void VisClient::drawLine(const Point& first, const Point& second) {
    if (!valid) return;
    ostringstream ss;
    ss << "drawLine " << first.x << " " << first.y << " " << second.x << " " << second.y;
    send(ss.str());
}

void VisClient::drawRect(const Rect& rectangle) {
    if (!valid) return;
    auto& points = rectangle.points();
    for (size_t i = 0; i < points.size(); i++) {
        drawLine(points[i], points[i + 1 == points.size() ? 0 : i + 1]);
    }
}

void VisClient::drawCircle(const Point& center, double radius) {
    if (!valid) return;
    ostringstream ss;
    ss << "drawCircle " << center.x << " " << center.y << " " << radius;
    send(ss.str());
}

void VisClient::drawText(const Point& point, const string& text) {
    if (!valid) return;
    ostringstream ss;
    ss << "drawText " << point.x << " " << point.y << " " << text;
    send(ss.str());
}

void VisClient::drawTextStatic(const Point& point, const string& text) {
    if (!valid) return;
    ostringstream ss;
    ss << "drawTextStatic " << (int) point.x << " " << (int) point.y << " " << text;
    send(ss.str());
}
