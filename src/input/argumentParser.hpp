#pragma once

#include <string>
#include <simulation/data/vec3d.hpp>

namespace ElectronOptics::Input
{
    
    class ArgumentParser {
        
    public:
        ArgumentParser(int argc, char** argv);
        bool shouldRunSolver() const { return m_configuration.runSolver; }
        std::string getFieldSolverConfigFilePath() const { return m_configuration.fieldSolverConfigFilePath; }
        bool shouldRunTracer() const { return m_configuration.runTracer; }
        std::string getTracerConfigFilePath() const { return m_configuration.tracerConfigFilePath; }
        std::string getOutputPath() const { return m_configuration.outputPath; }
        bool isVerbose() const { return m_configuration.verbose; }
    private:
        struct Configuration {
            bool runSolver{false};
            std::string fieldSolverConfigFilePath{""};
            bool runTracer{false};
            std::string tracerConfigFilePath{""};
            std::string outputPath{""};
            bool verbose{false};
        } m_configuration;

    };
    
} // namespace ElectronOptics::Input
