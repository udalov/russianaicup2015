#include "Runner.h"

#include <cstdlib>
#include <iostream>
#include <thread>

using std::string;

string javaHome = "/Library/Java/JavaVirtualMachines/jdk1.8.0_60.jdk/Contents/Home";

void runLocalRunner(bool vis) {
    string command = javaHome + "/bin/java -jar lib/local-runner.jar ";
    if (vis) command += "lib/local-runner.properties";
    else command += "lib/local-runner-console.properties";
    std::system(command.c_str());

    std::system("cat out/result.txt");
}

int main(int argc, char *argv[]) {
    auto startTime = clock();

    bool vis = false;
    for (int i = 1; i < argc; i++) {
        vis |= string("-vis") == argv[i];
    }

    auto localRunner = std::thread(runLocalRunner, vis);

    Runner runner("127.0.0.1", "31001", "0000000000000000");
    runner.run();
    localRunner.join();

    auto finishTime = clock();

    fprintf(stderr, "CPU time: %.3lf\n", (finishTime - startTime) * 1.0 / CLOCKS_PER_SEC);
}
