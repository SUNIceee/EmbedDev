/* Host/C11 equivalents for ARM assembly functions in funcs1.s. */

#include "6_generated_code.h"

int func2(int R0)
{
    /* ADD R0, R0, #1 with 32-bit wrapping semantics. */
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0)
{
    /* Transparent call to func2. */
    return func2(R0);
}

