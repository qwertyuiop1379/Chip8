#pragma once

#include "CPU.h"

namespace Chip8
{
    class Interpreter {
        public:
            struct Options {
                uint32_t memorySize;
                uint8_t stackEntryCount;
                uint8_t displayWidth;
                uint8_t displayHeight;

                Options() : memorySize(0x1000), stackEntryCount(0x10), displayWidth(64), displayHeight(32) {};
            };

        private:
            CPU *cpu;

        public:
            // Constructor/destructor
            Interpreter(const uint8_t * const rom, const size_t romSize, const Options &options = Options());
            ~Interpreter();

            // CPU
            const CPU * const GetCPU() const;

            // Controls
            void Start();

            // Display
            void Display();
    };
}