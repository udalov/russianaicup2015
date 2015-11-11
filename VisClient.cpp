#include <iostream>
#include <vector>
#include <sstream>
#include "VisClient.h"

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
    socket.Initialize();
    socket.DisableNagleAlgoritm();

    if (!socket.Open((uint8 *) "localhost", (int16) port)) {
        valid = false;
        cerr << "no visualization" << endl;
        return;
    }

    valid = true;
    cerr << "vis client socket opened on port " << port << endl;
}

void VisClient::drawLine(double x1, double y1, double x2, double y2) {
    ostringstream ss;
    ss << "line " << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
    send(ss.str());
}
