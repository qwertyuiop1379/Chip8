#include <stdio.h>
#include "util.h"

#include "Interpreter.h"
using namespace Chip8;

int main(int argc, char **argv)
{
    if (argc != 2) {
        printf("%s", string_format("Usage:\n\t%s <rom file>\n", argv[0]).c_str());
        return 1;
    }

    FILE *handle = fopen(argv[1], "rb");

    fseek(handle, 0, SEEK_END);
    size_t size = ftell(handle);
    fseek(handle, 0, SEEK_SET);

    uint8_t *bytes = (uint8_t *)malloc(size);
    fread(bytes, 1, size, handle);
    fclose(handle);

    Interpreter *in = new Interpreter(bytes, size);
    free(bytes);

    in->Start();

    delete in;
    return 0;
}