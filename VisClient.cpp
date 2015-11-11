#include <iostream>
#include <vector>
#include "VisClient.h"

using std::string;
using std::vector;

void writeBytes(CActiveSocket& socket, const vector<char>& bytes) {
    auto byteCount = bytes.size();
    unsigned int offset = 0;
    int sentByteCount;

    while (offset < byteCount && (sentByteCount = socket.Send((uint8*) &bytes[offset], byteCount - offset)) > 0) {
        offset += sentByteCount;
    }

    if (offset != byteCount) {
        std::cerr << "failed to send: " << offset << " != " << byteCount << std::endl;
    }
}

void writeString(CActiveSocket& socket, const string& str) {
    auto len = str.length();
    if (len > 65535) {
        std::cerr << "cannot send very large string (" << len << " chars): " << str << std::endl;
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
        std::cerr << "no visualization" << std::endl;
        return;
    }

    valid = true;
    std::cerr << "vis client socket opened on port " << port << std::endl;
}
