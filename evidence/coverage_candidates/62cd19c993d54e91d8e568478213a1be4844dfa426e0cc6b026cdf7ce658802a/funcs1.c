/* C implementation for assembly functions func1 and func2 */

#include "6_generated_code.h"

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}
