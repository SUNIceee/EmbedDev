/* DOCSTRING: Deck discovery mock and frozen log object definitions. */
#include "6_generated_code.h"

StateEstimateLog stateEstimate = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
Axis3Log gyro = {0.0f, 0.0f, 0.0f};
Axis3Log acc = {0.0f, 0.0f, 0.0f};
BaroLog baro = {0.0f, 0.0f, 0.0f};
MotorLog motor = {0u, 0u, 0u, 0u};
Sensfusion6Log sensfusion6Log = {1.0f, 0.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 1.0f, 0.0f,
                                 false, false};
SupervisorLog supervisorLog = {0u, 0.0f};
HealthLog healthLog = {0u, 0u, 0.0f, 0u};

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (decks == NULL || capacity == 0u) {
        return 0u;
    }

    /* Host model contains an empty deterministic deck inventory unless a
       platform adapter injects one through a private test fixture. */
    (void)decks;
    return 0u;
}
