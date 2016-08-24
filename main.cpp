#include "Runner.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace std;

#ifdef _LINUX
string javaHome = "/usr";
#else
string javaHome = "/Applications/IDEA.app/Contents/jre/jdk/Contents/Home/jre";
#endif

void runLocalRunner(bool vis) {
    string java = javaHome + "/bin/java";
    ifstream checkJavaExists(java);
    if (!checkJavaExists.good()) {
        cerr << "java executable not found at: " << java << endl;
        terminate();
    }

    string command = java + " -jar lib/local-runner.jar ";
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

    cerr << "CPU time: " << setprecision(3) << (finishTime - startTime) * 1.0 / CLOCKS_PER_SEC << endl;
}
