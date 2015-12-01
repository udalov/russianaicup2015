#include "Runner.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>

using namespace std;

string javaHome = "/Library/Java/JavaVirtualMachines/jdk1.8.0_60.jdk/Contents/Home";

void runLocalRunner(bool vis) {
    string command = "$(/usr/libexec/java_home -v 1.8)/bin/java -jar lib/local-runner.jar ";
    if (vis) command += "lib/local-runner.properties";
    else command += "lib/local-runner-console.properties";
    system(command.c_str());

    ifstream in("out/result.txt");
    string line;
    while (getline(in, line)) {
        cout << line << endl;
    }
}

int main(int argc, char *argv[]) {
    auto startTime = clock();

    bool vis = false;
    for (int i = 1; i < argc; i++) {
        vis |= string("-vis") == argv[i];
    }

    auto localRunner = thread(runLocalRunner, vis);

    Runner runner("127.0.0.1", "31001", "0000000000000000");
    runner.run();
    localRunner.join();

    auto finishTime = clock();

    fprintf(stderr, "CPU time: %.3lf\n", (finishTime - startTime) * 1.0 / CLOCKS_PER_SEC);
}
