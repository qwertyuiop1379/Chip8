#include "CPU.h"
#include "util.h"

#include <stdexcept>
#include <stdlib.h>
#include <time.h>

namespace Chip8
{
    std::string DecodeInstruction(const uint16_t instruction)
    {
        return "null";
    }

    CPU::CPU(uint32_t memorySize, uint8_t stackEntryCount, uint8_t displayWidth, uint8_t displayHeight, void (*logFunction)(const std::string &))
    {
        srand(time(NULL));

        this->logFunction = logFunction;
        this->LogF("Initialize CPU: RAM: 0x%x bytes; Stack entries: 0x%x; Display: %dx%d.", memorySize, stackEntryCount, displayWidth, displayHeight);

        this->displayWidth = displayWidth;
        this->displayHeight = displayHeight;

        auto pixels = displayWidth * displayHeight;
        auto frameBufferSize = (pixels / 8) + (pixels % 8 > 0);

        this->frameBuffer = (uint8_t *)malloc(frameBufferSize);
        this->ClearDisplay();

        this->memorySize = memorySize;
        this->memory = (uint8_t *)malloc(memorySize);

        this->stackEntryCount = stackEntryCount;
        this->stack = (uint16_t *)malloc(stackEntryCount * sizeof(uint16_t));
    }

    CPU::~CPU()
    {
        this->LogF("Destruct CPU.");

        free(this->frameBuffer);
        free(this->memory);
        free(this->stack);
    }

    void CPU::UnknownInstruction(uint16_t instruction)
    {
        throw std::runtime_error(string_format("Unknown instruction 0x%04x. Exiting.", instruction));
    }

    template<typename ... Args>
    void CPU::LogF(const std::string &format, Args ... args) const
    {
        if (this->logFunction != nullptr)
            this->logFunction(string_format("[CPU] " + format, args ...));
    }

    const uint8_t CPU::Read8(const uint16_t address) const
    {
        if (address > this->memorySize)
            throw std::runtime_error(string_format("Error accessing Chip-8 RAM: address 0x%x is out of bounds.", address));

        uint8_t value = this->memory[address];
        this->LogF("Read 0x%02x from address 0x%04x.", value, address);
        return value;
    }

    const uint16_t CPU::Read16(const uint16_t address) const
    {
        if (address + 1 > this->memorySize)
            throw std::runtime_error(string_format("Error accessing Chip-8 RAM: address 0x%x is out of bounds.", address + 1));

        uint16_t value = (this->memory[address] << 8) | this->memory[address + 1];
        this->LogF("Read 0x%04x from address 0x%04x.", value, address);
        return value;
    }

    void CPU::Write8(const uint16_t address, const uint8_t value)
    {
        if (address > this->memorySize)
            throw std::runtime_error(string_format("Error accessing Chip-8 RAM: address 0x%x is out of bounds.", address));
            
        this->LogF("Write 0x%02x to address 0x%04x.", value, address);
        this->memory[address] = value;
    }

    void CPU::Write16(const uint16_t address, const uint16_t value)
    {
        if (address + 1 > this->memorySize)
            throw std::runtime_error(string_format("Error accessing Chip-8 RAM: address 0x%x is out of bounds.", address + 1));

        this->LogF("Write 0x%04x to address 0x%04x.", value, address);
        this->memory[address] = value >> 8;
        this->memory[address + 1] = value & 0xFF;
    }

    void CPU::WriteBytes(const uint16_t address, const uint8_t *bytes, const size_t size)
    {
        if (address + size > this->memorySize)
            throw std::runtime_error(string_format("Error accessing Chip-8 RAM: address 0x%x is out of bounds.", address + size));

        this->LogF("Write 0x%zx bytes to address 0x%04x.", size, address);
        std::memcpy(this->memory + address, bytes, size);
    }

    const uint16_t CPU::ReadPC() const
    {
        uint16_t value = this->registers.pc;
        this->LogF("Read 0x%04x from register PC.", value);
        return value;
    }

    const uint8_t CPU::ReadSP() const
    {
        uint8_t value = this->registers.sp;
        this->LogF("Read 0x%02x from register SP.", value);
        return value;
    }

    const uint8_t CPU::ReadV(uint8_t index) const
    {
        if (index > 0x10)
            throw std::runtime_error(string_format("Error accessing Chip-8 CPU register: register V%x does not exist.", index));

        uint8_t value = this->registers.v[index];
        this->LogF("Read 0x%02x from register V%x.", value, index);
        return value;
    }

    const uint16_t CPU::ReadI() const
    {
        uint16_t value = this->registers.I;
        this->LogF("Read 0x%04x from register I.", value);
        return value;
    }

    const uint8_t CPU::ReadDelayTimer() const
    {
        uint8_t value = this->registers.delayTimer;
        this->LogF("Read 0x%02x from delay timer register.", value);
        return value;
    }

    const uint8_t CPU::ReadSoundTImer() const
    {
        uint8_t value = this->registers.soundTimer;
        this->LogF("Read 0x%02x from sound timer register.", value);
        return value;
    }

    void CPU::WritePC(const uint16_t address)
    {
        if (address + 1 > this->memorySize)
            throw std::runtime_error(string_format("Error writing Chip-8 CPU program counter: address 0x%x is out of bounds.", address + 1));

        this->LogF("Write address 0x%04x to register PC.", address);
        this->registers.pc = address;
    }

    void CPU::WriteSP(const uint8_t index)
    {
        if (index > this->stackEntryCount)
            throw std::runtime_error(string_format("Error writing Chip-8 CPU stack pointer: index 0x%x exceeds stack entry count (0x%x).", index, this->stackEntryCount));

        this->LogF("Write 0x%02x to register SP.", index);
        this->registers.sp = index;
    }

    void CPU::WriteV(const uint8_t index, const uint8_t value)
    {
        if (index > 0x10)
            throw std::runtime_error(string_format("Error accessing Chip-8 CPU register: register V%x does not exist.", index));

        this->LogF("Write 0x%02x to register V%x.", value, index);
        this->registers.v[index] = value;
    }

    void CPU::WriteI(const uint16_t address)
    {
        this->LogF("Write address 0x%04x to register I", address);
        this->registers.I = address;
    }

    void CPU::WriteDelayTimer(const uint8_t timer)
    {
        this->LogF("Write 0x%02x to delay timer register.", timer);
        this->registers.delayTimer = timer;
    }

    void CPU::WriteSoundTimer(const uint8_t timer)
    {
        this->LogF("Write 0x%02x to sound timer register.", timer);
        this->registers.soundTimer = timer;
    }

    const uint16_t CPU::Pop()
    {
        uint8_t sp = this->ReadSP() - 1;
        uint16_t address = this->stack[sp];

        this->LogF("Read address 0x%04x from stack index %d.", address, sp);
        this->WriteSP(sp);

        return address;
    }

    void CPU::Push(const uint16_t address)
    {
        uint8_t sp = this->ReadSP();
        this->LogF("Push address 0x%04x to stack index %d.", address, sp);
        
        this->stack[sp] = address;
        this->WriteSP(sp + 1);
    }

    void CPU::ExecuteCycle()
    {
        this->LogF("Begin cycle.");
        
        uint16_t pc = this->ReadPC();
        uint16_t instruction = this->Read16(pc);

        this->WritePC(pc + 2);
        this->ExecuteInstruction(instruction);
    }

    void CPU::ExecuteInstruction(const uint16_t instruction)
    {
        std::string decode = DecodeInstruction(instruction);
        this->LogF("Execute instruction 0x%04x (%s).", instruction, decode.c_str());

        switch (instruction >> 12) {
            case 0x0: {
                if (instruction == 0x00E0) { // CLS
                    this->LogF("Clear display.");
                    this->ClearDisplay();
                } else if (instruction == 0x00EE) { // RET
                    uint16_t address = this->Pop();
                    this->LogF("Return from subroutine to address 0x%x.", address);
                    this->WritePC(address);
                } else { // SYS addr
                    throw std::runtime_error("CPU cannot execute SYS instruction. Exiting.");
                }

                break;
            }

            case 0x1: { // JP addr
                uint16_t address = instruction & 0x0FFF;
                this->LogF("Jump to address 0x%x.", address);
                this->WritePC(address);
                break;
            }

            case 0x2: { // CALL addr
                uint16_t address = instruction & 0x0FFF;
                this->LogF("Call subroutine at address 0x%x.", address);
                this->Push(this->ReadPC());
                this->WritePC(address);
                break;
            }

            case 0x3: { // SE Vx, byte
                uint8_t index = (instruction & 0x0F00) >> 8;
                uint8_t byte = instruction & 0x00FF;
                uint8_t value = this->ReadV(index);

                if (value == byte) {
                    this->LogF("Skip next instruction.");
                    uint16_t pc = this->ReadPC();
                    this->WritePC(pc + 2);
                }
                
                break;
            }

            case 0x4: { // SNE Vx, byte
                uint8_t index = (instruction & 0x0F00) >> 8;
                uint8_t byte = instruction & 0x00FF;
                uint8_t value = this->ReadV(index);

                if (value != byte) {
                    this->LogF("Skip next instruction.");
                    uint16_t pc = this->ReadPC();
                    this->WritePC(pc + 2);
                }
                
                break;
            }

            case 0x5: { // SE Vx, Vy
                if (instruction & 0x000F) {
                    this->UnknownInstruction(instruction);
                    break;
                }

                uint8_t x = (instruction & 0x0F00) >> 8;
                uint8_t y = (instruction & 0x00F0) >> 4;
                uint8_t v1 = this->ReadV(x);
                uint8_t v2 = this->ReadV(y);

                if (v1 == v2) {
                    this->LogF("Skip next instruction.");
                    uint16_t pc = this->ReadPC();
                    this->WritePC(pc + 2);
                }
                
                break;
            }

            case 0x6: { // LD Vx, byte
                uint8_t index = (instruction & 0x0F00) >> 8;
                uint8_t byte = instruction & 0x00FF;
                this->WriteV(index, byte);
                break;
            }

            case 0x7: { // ADD Vx, byte
                uint8_t index = (instruction & 0x0F00) >> 8;
                uint8_t byte = instruction & 0x00FF;
                uint8_t value = this->ReadV(index);
                this->LogF("Add 0x%x to V%x (0x%x). New value: 0x%x.", byte, index, value, value + byte);
                this->WriteV(index, value + byte);
                break;
            }

            case 0x8: {
                uint8_t n = instruction & 0x000F;
                uint8_t x = (instruction & 0x0F00) >> 8;
                uint8_t y = (instruction & 0x00F0) >> 4;

                switch (n) {
                    case 0x0: { // LD Vx, Vy
                        uint8_t value = this->ReadV(y);
                        this->WriteV(x, value);
                        break;
                    }

                    case 0x1: { // OR Vx, Vy
                        uint8_t v1 = this->ReadV(y);
                        uint8_t v2 = this->ReadV(x);
                        this->WriteV(x, v1 | v2);
                        break;
                    }

                    case 0x2: { // AND Vx, Vy
                        uint8_t v1 = this->ReadV(y);
                        uint8_t v2 = this->ReadV(x);
                        this->WriteV(x, v1 & v2);
                        break;
                    }

                    case 0x3: { // XOR Vx, Vy
                        uint8_t v1 = this->ReadV(y);
                        uint8_t v2 = this->ReadV(x);
                        this->WriteV(x, v1 ^ v2);
                        break;
                    }

                    case 0x4: { // ADD Vx, Vy
                        uint8_t v1 = this->ReadV(y);
                        uint8_t v2 = this->ReadV(x);
                        uint16_t sum = v1 + v2;
                        this->WriteV(x, sum & 0xFF);
                        this->WriteV(0xF, sum > 0xFF);
                        break;
                    }

                    case 0x5: { // SUB Vx, Vy
                        uint8_t v1 = this->ReadV(y);
                        uint8_t v2 = this->ReadV(x);
                        this->WriteV(x, v2 - v1);
                        this->WriteV(0xF, v2 >= v1);
                        break;
                    }

                    case 0x6: { // SHR Vx, {, Vy}
                        uint8_t value = this->ReadV(y);
                        this->WriteV(x, value >> 1);
                        this->WriteV(y, value >> 1);
                        this->WriteV(0xF, value & 1);
                        break;
                    }

                    case 0x7: { // SUBN Vx, Vy
                        uint8_t v1 = this->ReadV(y);
                        uint8_t v2 = this->ReadV(x);
                        this->WriteV(x, v1 - v2);
                        this->WriteV(0xF, v1 >= v2);
                        break;
                    }

                    case 0xE: { // SHL Vx, {, Vy}
                        uint8_t value = this->ReadV(y);
                        this->WriteV(x, value << 1);
                        this->WriteV(y, value << 1);
                        this->WriteV(0xF, value >> 7);
                        break;
                    }

                    default: this->UnknownInstruction(instruction); break;
                }

                break;
            }

            case 0x9: { // SNE Vx, Vy
                if (instruction & 0x000F) {
                    this->UnknownInstruction(instruction);
                    break;
                }

                uint8_t x = (instruction & 0x0F00) >> 8;
                uint8_t y = (instruction & 0x00F0) >> 4;
                uint8_t v1 = this->ReadV(x);
                uint8_t v2 = this->ReadV(y);

                if (v1 != v2) {
                    this->LogF("Skip next instruction.");
                    uint16_t pc = this->ReadPC();
                    this->WritePC(pc + 2);
                }
                
                break;
            }

            case 0xA: { // LD I, addr
                uint16_t address = instruction & 0x0FFF;
                this->WriteI(address);
                break;
            }

            case 0xB: { // JP V0, addr
                uint16_t address = (instruction & 0x0FFF) + this->ReadV(0);
                this->LogF("Jump to address 0x%x.", address);
                this->WritePC(address);
                break;
            }

            case 0xC: { // RND Vx, byte
                uint8_t index = (instruction & 0x0F00) >> 8;
                uint8_t byte = instruction & 0x00FF;
                uint8_t value = rand() & byte;
                this->LogF("Write random number (0x%x) to V%x.", value, index);
                this->WriteV(index, value);
                break;
            }

            case 0xD: { // DRW Vx, Vy, nibble
                uint8_t vx = (instruction & 0x0F00) >> 8;
                uint8_t vy = (instruction & 0x00F0) >> 4;
                uint8_t n = instruction & 0x000F;

                uint16_t address = this->ReadI();
                uint8_t x = this->ReadV(vx);
                uint8_t y = this->ReadV(vy);

                this->LogF("Draw 0x%x bytes from address 0x%04x at (%d, %d).", n, address, x, y);

                bool collision = false;
                bool setVF = false;
                
                for (int i = 0; i < n; i++) {
                    uint8_t byte = this->memory[address++];

                    for (int b = 0; b < 8; b++) {
                        bool on = (byte & (1 << (7 - b))) > 0;
                        auto dx = (x + b) % this->displayWidth;
                        auto dy = (y + i) % this->displayHeight;

                        this->SetPixel(dx, dy, on, &collision);

                        if (collision)
                            setVF = true;
                    }
                }
                
                this->WriteV(0xF, setVF);

                break;
            }

            case 0xE: {
                uint8_t n = instruction & 0xFF;
                switch (n) {
                    case 0x9E: { // SKP Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint8_t key = this->ReadV(index);

                        if (this->ReadKey(key)) {
                            this->LogF("Skip next instruction.");
                            uint16_t pc = this->ReadPC();
                            this->WritePC(pc + 2);
                        }

                        break;
                    }

                    case 0xA1: { // SKNP Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint8_t key = this->ReadV(index);

                        if (!this->ReadKey(key)) {
                            this->LogF("Skip next instruction.");
                            uint16_t pc = this->ReadPC();
                            this->WritePC(pc + 2);
                        }

                        break;
                    }

                    default: this->UnknownInstruction(instruction); break;
                }

                break;
            }

            case 0xF: {
                uint8_t n = instruction & 0x00FF;
                switch (n) {
                    case 0x07: { // LD Vx, DT
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint8_t timer = this->ReadDelayTimer();
                        this->LogF("Read delay timer to V%x.", index);
                        this->WriteV(index, timer);
                        break;
                    }

                    case 0x0A: { // LD Vx, K
                        throw std::runtime_error("key press");
                        break;
                    }

                    case 0x15: { // LD DT, Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint8_t timer = this->ReadV(index);
                        this->LogF("Write V%x (0x%x) to delay timer.", index, timer);
                        this->WriteDelayTimer(timer);
                        break;
                    }

                    case 0x18: {// LD ST, Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint8_t timer = this->ReadV(index);
                        this->LogF("Write V%x (0x%x) to sound timer.", index, timer);
                        this->WriteSoundTimer(timer);
                        break;
                    }

                    case 0x1E: { // ADD I, Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint16_t address = this->ReadI() + this->ReadV(index);
                        this->LogF("Add V%x to I.", index);
                        this->WriteI(address);
                        break;
                    }

                    case 0x29: { // LD F, Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint8_t digit = this->ReadV(index);
                        uint16_t address = digit * 5;
                        this->LogF("Write address of digit '%d' (%04x) to I.", digit, address);
                        this->WriteI(digit);
                        break;
                    }

                    case 0x33: { // LD B, Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint8_t value = this->ReadV(index);
                        uint16_t address = this->ReadI();

                        this->LogF("Store BCD of 0x%x at address 0x%04x-0x%04x. 100: %d, 10: %d, 1: %d.", value, address, address + 2, value / 100, (value % 100) / 10, value % 10);
                        this->Write8(address, value / 100);
                        this->Write8(address + 1, (value % 100) / 10);
                        this->Write8(address + 2, value % 10);
                        break;
                    }

                    case 0x55: { // LD [I], Vx
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint16_t address = this->ReadI();

                        this->LogF("Store V0-V%x to address 0x%04x.", index, address);

                        for (int i = 0; i <= index; i++) {
                            uint8_t value = this->ReadV(i);
                            this->Write8(address, value);
                            address++;
                        }

                        this->WriteI(address);
                        break;
                    }

                    case 0x65: { // LD Vx, [I]
                        uint8_t index = (instruction & 0x0F00) >> 8;
                        uint16_t address = this->ReadI();

                        this->LogF("Load V0-V%x from address 0x%04x.", index, address);

                        for (int i = 0; i <= index; i++) {
                            uint8_t value = this->Read8(address);
                            this->WriteV(i, value);
                            address++;
                        }

                        this->WriteI(address);
                        break;
                    }

                    default: this->UnknownInstruction(instruction); break;
                }
                break;
            }

            default: this->UnknownInstruction(instruction); break;
        }

        this->LogF("Cycle complete.\n");
    }

    const bool CPU::ReadKey(const uint8_t key) const
    {
        return (1 << key) & this->keys;
    }

    void CPU::WriteKey(const uint8_t key, const bool pressed)
    {
        this->keys = (this->keys & ~((uint8_t)1 << key)) | ((uint8_t)pressed << key);
    }

    void CPU::ClearDisplay()
    {
        auto pixels = this->displayWidth * this->displayHeight;
        auto size = (pixels / 8) + (pixels % 8 > 0);
        std::memset(this->frameBuffer, 0, size);
    }
    
    const bool CPU::ReadPixel(const uint8_t x, const uint8_t y) const
    {
        auto pixel = this->displayWidth * y + x;
        auto byte = this->frameBuffer[pixel / 8];
        bool on = (byte & (1 << x % 8)) > 0;
        this->LogF("Read pixel (%d, %d): %d.", x, y, on);
        return on;
    }

    void CPU::SetPixel(const uint8_t x, const uint8_t y, bool on, bool *collision)
    {
        auto pixel = this->displayWidth * y + x;
        auto byte = this->frameBuffer[pixel / 8];
        auto mx = x % 8;

        bool collide = ((byte & (1 << mx)) > 0) && on;

        if (collision != nullptr)
            *collision = collide;

        this->LogF("Write pixel (%d, %d): %d. Collision: %d.", x, y, on, collide);
        byte ^= (uint8_t)on << mx;
        this->frameBuffer[pixel / 8] = byte;
    }

    const uint8_t CPU::GetDisplayWidth() const
    {
        return this->displayWidth;
    }

    const uint8_t CPU::GetDisplayHeight() const
    {
        return this->displayHeight;
    }
}