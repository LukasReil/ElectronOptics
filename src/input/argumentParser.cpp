#include "argumentParser.hpp"

#include <stdexcept>
#include <iostream>
#include <spdlog/spdlog.h>

ElectronOptics::Input::ArgumentParser::ArgumentParser(int argc, char **argv) {
    int currentArgumentIndex = 1;

    while (currentArgumentIndex < argc) {
        std::string currentArgument = argv[currentArgumentIndex];

        if (currentArgument == "-s" || currentArgument == "--run-solver") {
            if (currentArgumentIndex + 1 < argc) {
                currentArgumentIndex++;
                m_configuration.runSolver = true;
                m_configuration.fieldSolverConfigFilePath = argv[currentArgumentIndex];
            } else {
                throw std::runtime_error("Expected configuration file path after " + currentArgument);
            }
        } else if (currentArgument == "-t" || currentArgument == "--run-tracer") {
            if (currentArgumentIndex + 1 < argc) {
                currentArgumentIndex++;
                m_configuration.runTracer = true;
                m_configuration.tracerConfigFilePath = argv[currentArgumentIndex];
            } else {
                throw std::runtime_error("Expected configuration file path after " + currentArgument);
            }
        } else if (currentArgument == "-o" || currentArgument == "--output") {
            if (currentArgumentIndex + 1 < argc) {
                currentArgumentIndex++;
                m_configuration.outputPath = argv[currentArgumentIndex];
            } else {
                throw std::runtime_error("Expected output path after " + currentArgument);
            }
        } else if (currentArgument == "-v" || currentArgument == "--verbose") {
            m_configuration.verbose = true;
        } else if (currentArgument == "-g" || currentArgument == "--gui") {
            spdlog::warn("GUI mode is not implemented yet. This flag will be ignored.");
        } else if (currentArgument == "--help") {
            printHelp();
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + currentArgument);
        }
    }
}

void printHelp() {
    std::cout << "Usage: ElectronOptics [options]\n"
              << "Options:\n"
              << "  -s, --run-solver <file>         Runs the FEM solver with the specified configuration file\n"
              << "  -t, --run-tracer <file>         Runs the electron tracer with the specified configuration file.\n"
              << "                                  If solver results are available, they will be used for tracing. Otherwise, the tracer will attempt to load a potential mesh from the specified configuration file.\n"
              << "  -o, --output <path>             Path to the output directory for results\n"
              << "  -v, --verbose                   Enable verbose logging\n"
              << "  -g, --gui                       Run the application with a graphical user interface\n"
              << "  --help                          Show this help message\n";
}