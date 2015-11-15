#include "VisClient.h"

#include <iostream>
#include <vector>
#include <sstream>

using namespace std;

void writeBytes(CActiveSocket& socket, const vector<char>& bytes) {
    auto byteCount = bytes.size();
    unsigned int offset = 0;
    int sentByteCount;

    while (offset < byteCount && (sentByteCount = socket.Send((uint8*) &bytes[offset], byteCount - offset)) > 0) {
        offset += sentByteCount;
    }

    if (offset != byteCount) {
        cerr << "failed to send: " << offset << " != " << byteCount << endl;
    }
}

void writeString(CActiveSocket& socket, const string& str) {
    auto len = str.length();
    if (len > 65535) {
        cerr << "cannot send very large string (" << len << " chars): " << str << endl;
        return;
    }

    vector<char> bytes;
    bytes.push_back((char) ((len >> 8) & 0xFF));
    bytes.push_back((char) (len & 0xFF));

    bytes.insert(bytes.end(), str.begin(), str.end());
    writeBytes(socket, bytes);
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
    cerr << "vis client socket opened on port " << port << endl;
}

void VisClient::drawLine(const Point& first, const Point& second) {
    ostringstream ss;
    ss << "line " << first.x << " " << first.y << " " << second.x << " " << second.y;
    send(ss.str());
}

void VisClient::drawPoly(const vector<Point>& points) {
    for (auto i = 0UL; i < points.size(); i++) {
        drawLine(points[i], points[i + 1 == points.size() ? 0 : i + 1]);
    }
}

void VisClient::drawRect(const Rect& rectangle) {
    drawPoly(rectangle.points);
}

void VisClient::drawCircle(const Point& center, double radius) {
    ostringstream ss;
    ss << "circle " << center.x << " " << center.y << " " << radius;
    send(ss.str());
}
