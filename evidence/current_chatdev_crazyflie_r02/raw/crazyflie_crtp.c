/* DOCSTRING: Bounded CRTP TX/RX queues, port callbacks, link mocks, and statistics. */
#include "6_generated_code.h"
#include "crazyflie_internal.h"

#include <string.h>

static bool NopSendPacket(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool NopReceivePacket(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool NopIsConnected(void)
{
    return true;
}

static void NopSetEnable(bool enable)
{
    (void)enable;
}

static void NopReset(void)
{
}

static CrtpLink s_nopLink = {
    NopSendPacket,
    NopReceivePacket,
    NopIsConnected,
    NopSetEnable,
    NopReset
};

static CrtpPacket s_txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t s_txHead = 0u;
static uint16_t s_txTail = 0u;
static uint16_t s_txCount = 0u;

static CrtpPacket s_rxQueue[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint8_t s_rxHead[CRTP_NBR_OF_PORTS];
static uint8_t s_rxTail[CRTP_NBR_OF_PORTS];
static uint8_t s_rxCount[CRTP_NBR_OF_PORTS];
static bool s_rxActive[CRTP_NBR_OF_PORTS];

static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS];

static CrtpLink *s_currentLink = &s_nopLink;
static bool s_crtpInitialized = false;
static bool s_crtpError = false;

static uint32_t s_lastStatsTick = 0u;
static uint32_t s_rxPacketsSinceLast = 0u;
static uint32_t s_txPacketsSinceLast = 0u;

void crtpInit(void)
{
    if (s_crtpInitialized) {
        return;
    }

    memset(s_txQueue, 0, sizeof(s_txQueue));
    s_txHead = 0u;
    s_txTail = 0u;
    s_txCount = 0u;

    memset(s_rxQueue, 0, sizeof(s_rxQueue));
    memset(s_rxHead, 0, sizeof(s_rxHead));
    memset(s_rxTail, 0, sizeof(s_rxTail));
    memset(s_rxCount, 0, sizeof(s_rxCount));
    memset(s_rxActive, 0, sizeof(s_rxActive));
    memset(s_portCallbacks, 0, sizeof(s_portCallbacks));

    s_currentLink = &s_nopLink;
    s_crtpError = false;
    s_lastStatsTick = g_systemTickMs;
    s_rxPacketsSinceLast = 0u;
    s_txPacketsSinceLast = 0u;

    s_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        s_crtpError = true;
        return;
    }

    if (s_rxActive[port]) {
        s_crtpError = true;
        return;
    }

    s_rxActive[port] = true;
    s_rxHead[port] = 0u;
    s_rxTail[port] = 0u;
    s_rxCount[port] = 0u;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (packet == NULL || s_txCount >= CRTP_TX_QUEUE_SIZE) {
        return false;
    }

    s_txQueue[s_txTail] = *packet;
    s_txTail = (uint16_t)((s_txTail + 1u) % CRTP_TX_QUEUE_SIZE);
    s_txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

static bool dequeueRx(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !s_rxActive[port] ||
        s_rxCount[port] == 0u || packet == NULL) {
        return false;
    }

    *packet = s_rxQueue[port][s_rxHead[port]];
    s_rxHead[port] = (uint8_t)((s_rxHead[port] + 1u) % CRTP_RX_QUEUE_SIZE);
    s_rxCount[port]--;
    return true;
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    return dequeueRx(port, packet);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    return dequeueRx(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet,
                           uint32_t wait_ms)
{
    (void)wait_ms;
    return dequeueRx(port, packet);
}

static void pushRxPacket(uint8_t port, const CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !s_rxActive[port] ||
        packet == NULL || s_rxCount[port] >= CRTP_RX_QUEUE_SIZE) {
        return;
    }

    s_rxQueue[port][s_rxTail[port]] = *packet;
    s_rxTail[port] = (uint8_t)((s_rxTail[port] + 1u) % CRTP_RX_QUEUE_SIZE);
    s_rxCount[port]++;
}

void crtpRxTask(void)
{
    if (s_currentLink == &s_nopLink || s_currentLink == NULL ||
        s_currentLink->receivePacket == NULL) {
        return;
    }

    CrtpPacket packet;
    uint8_t budget = CRTP_RX_QUEUE_SIZE;
    while (budget-- > 0u && s_currentLink->receivePacket(&packet)) {
        bool delivered = false;

        if (packet.port < CRTP_NBR_OF_PORTS &&
            s_rxActive[packet.port] &&
            s_rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
            pushRxPacket(packet.port, &packet);
            delivered = true;
        }

        if (packet.port < CRTP_NBR_OF_PORTS &&
            s_portCallbacks[packet.port] != NULL) {
            s_portCallbacks[packet.port](&packet);
            delivered = true;
        }

        if (!delivered) {
            /* Drop packet. */
        }

        s_rxPacketsSinceLast++;
    }
}

void crtpTxTask(void)
{
    if (s_currentLink == &s_nopLink || s_currentLink == NULL ||
        s_currentLink->sendPacket == NULL || s_txCount == 0u) {
        return;
    }

    CrtpPacket packet = s_txQueue[s_txHead];
    if (s_currentLink->sendPacket(&packet)) {
        s_txHead = (uint16_t)((s_txHead + 1u) % CRTP_TX_QUEUE_SIZE);
        s_txCount--;
        s_txPacketsSinceLast++;
    }
    /* On failure the packet is retained for the next retry. */
}

void crtpSetLink(CrtpLink *newLink)
{
    if (s_currentLink != NULL && s_currentLink->setEnable != NULL) {
        s_currentLink->setEnable(false);
    }

    if (newLink == NULL) {
        s_currentLink = &s_nopLink;
    } else {
        s_currentLink = newLink;
    }

    if (s_currentLink != NULL && s_currentLink->setEnable != NULL) {
        s_currentLink->setEnable(true);
    }
}

void crtpReset(void)
{
    s_txHead = 0u;
    s_txTail = 0u;
    s_txCount = 0u;

    if (s_currentLink != NULL && s_currentLink->reset != NULL) {
        s_currentLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (s_currentLink != NULL && s_currentLink->isConnected != NULL) {
        return s_currentLink->isConnected();
    }

    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - s_txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        s_crtpError = true;
        return;
    }

    s_portCallbacks[port] = callback;
}

void updateStats(void)
{
    if ((g_systemTickMs - s_lastStatsTick) >= 500u) {
        s_lastStatsTick = g_systemTickMs;
        s_rxPacketsSinceLast = 0u;
        s_txPacketsSinceLast = 0u;
    }
}
