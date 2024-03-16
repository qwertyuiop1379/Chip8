#include "Interpreter.h"

#include <chrono>
#include <thread>

namespace Chip8
{
    static bool display = true;

    static uint8_t digitROM[] = {
        0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
        0x20, 0x60, 0x20, 0x20, 0x70, // 1
        0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
        0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
        0x90, 0x90, 0xF0, 0x10, 0x10, // 4
        0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
        0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
        0xF0, 0x10, 0x20, 0x40, 0x40, // 7
        0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
        0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
        0xF0, 0x90, 0xF0, 0x90, 0x90, // A
        0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
        0xF0, 0x80, 0x80, 0x80, 0xF0, // C
        0xE0, 0x90, 0x90, 0x90, 0xE0, // D
        0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
        0xF0, 0x80, 0xF0, 0x80, 0x80  // F
    };

    void DebugLog(const std::string &message)
    {
        if (!display)
            printf("%s\n", message.c_str());
    }

    Interpreter::Interpreter(const uint8_t * const rom, const size_t romSize, const Options &options)
    {
        this->cpu = new CPU(options.memorySize, options.stackEntryCount, options.displayWidth, options.displayHeight, DebugLog);
        this->cpu->WriteBytes(0x200, rom, romSize);
        this->cpu->WriteBytes(0x0, digitROM, sizeof(digitROM));
        this->cpu->WritePC(0x200);
        this->cpu->LogF("Done initializing.\n");
    }

    Interpreter::~Interpreter()
    {
        delete this->cpu;
    }

    const CPU * const Interpreter::GetCPU() const { return this->cpu; }

    void Interpreter::Start()
    {
        // test code

        uint8_t framerate = 60;

        for (int i = 0; i < 1000; i++) {
            this->cpu->ExecuteCycle();

            uint8_t dt = this->cpu->ReadDelayTimer();
            if (dt != 0)
                this->cpu->WriteDelayTimer(dt - 1);

            if (display)
                Display();

            uint32_t ns = (1000 / framerate) * 1000000;
            std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
        }
    }

    void Interpreter::Display()
    {
        printf("\x1B[2J\033[0;0H");

        uint8_t width = this->cpu->GetDisplayWidth();
        uint8_t height = this->cpu->GetDisplayHeight();
        
        printf("X");
        for (int x = 0; x < width; x++)
            printf("-");
        printf("X\n");

        for (int y = 0; y < height; y++) {
            printf("|");
            for (int x = 0; x < width; x++) {
                bool on = this->cpu->ReadPixel(x, y);
                printf("%s", on ? "X" : " ");
            }

            printf("|\n");
        }

        printf("X");
        for (int x = 0; x < width; x++)
            printf("-");
        printf("X\n");
    }
}