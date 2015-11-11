#include <cstdlib>
#include <iostream>
#include <thread>
#include "Runner.h"

bool visualization_mode = true;

void run_local_runner() {
    chdir("/Users/udalov/c/russianaicup15");
    auto command = std::string { "java -jar lib/local-runner.jar " };
    if (visualization_mode) command += "lib/local-runner.properties";
    else command += "lib/local-runner-console.properties";
    std::system(command.c_str());
    std::system("cat out/result.txt");
}

int main() {
    auto begin_time = clock();

    auto local_runner_thread = std::thread(run_local_runner);

    Runner runner("127.0.0.1", "31001", "0000000000000000");
    runner.run();
    local_runner_thread.join();

    auto end_time = clock();

    fprintf(stderr, "CPU time: %.3lf\n", (end_time - begin_time) * 1.0 / CLOCKS_PER_SEC);
}
