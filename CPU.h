#pragma once

#include <stdint.h>
#include <string>

namespace Chip8
{
    class CPU {
        private:
            void (*logFunction)(const std::string &) = nullptr;

            uint32_t memorySize;
            uint8_t *memory;

            uint8_t stackEntryCount;
            uint16_t *stack;

            uint8_t displayWidth;
            uint8_t displayHeight;
            uint8_t *frameBuffer;

            struct {
                uint16_t pc;
                uint8_t sp;
                uint8_t v[0x10];
                uint16_t I;
                uint8_t delayTimer;
                uint8_t soundTimer;
            } registers;

            uint16_t keys;

            void UnknownInstruction(uint16_t instruction);

        public:
            // Constructor/destructor
            CPU(uint32_t memorySize, uint8_t stackEntryCount, uint8_t displayWidth, uint8_t displayHeight, void (*logFunction)(const std::string &) = nullptr);
            ~CPU();

            // Log
            template<typename ... Args>
            void LogF(const std::string &format, Args ... args) const;

            // Read memory
            const uint8_t Read8(const uint16_t address) const;
            const uint16_t Read16(const uint16_t address) const;

            // Write memory
            void Write8(const uint16_t address, const uint8_t value);
            void Write16(const uint16_t address, const uint16_t value);
            void WriteBytes(const uint16_t address, const uint8_t *bytes, const size_t size);

            // Read register
            const uint16_t ReadPC() const;
            const uint8_t ReadSP() const;
            const uint8_t ReadV(uint8_t index) const;
            const uint16_t ReadI() const;
            const uint8_t ReadDelayTimer() const;
            const uint8_t ReadSoundTImer() const;

            // Write register
            void WritePC(const uint16_t address);
            void WriteSP(const uint8_t index);
            void WriteV(const uint8_t index, const uint8_t value);
            void WriteI(const uint16_t address);
            void WriteDelayTimer(const uint8_t timer);
            void WriteSoundTimer(const uint8_t timer);

            // Stack
            const uint16_t Pop();
            void Push(const uint16_t address);

            // Execution
            void ExecuteCycle();
            void ExecuteInstruction(const uint16_t instruction);

            // Keyboard
            const bool ReadKey(const uint8_t key) const;
            void WriteKey(const uint8_t key, const bool pressed);

            // Display
            void ClearDisplay();
            const bool ReadPixel(const uint8_t x, const uint8_t y) const;
            void SetPixel(const uint8_t x, const uint8_t y, bool on, bool *collision = nullptr);
            const uint8_t GetDisplayWidth() const;
            const uint8_t GetDisplayHeight() const;
    };
}