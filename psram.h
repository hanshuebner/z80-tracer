/*
 * PSRAM driver for PGA2350 (APS6404L-3SQR, 8 MB QSPI PSRAM on GPIO 47).
 *
 * Based on AudioMorphology/PSRAM (MIT, originally from SparkFun),
 * adapted for this project.
 */

#ifndef PSRAM_H
#define PSRAM_H

#include <stddef.h>
#include <stdint.h>

/* Memory-mapped PSRAM addresses (CS1 window) */
#define PSRAM_BASE_CACHED    0x11000000u
#define PSRAM_BASE_UNCACHED  0x15000000u

/* CS pin for PGA2350 */
#define PSRAM_CS_PIN         47

/*
 * Initialise PSRAM via QMI.  Returns detected size in bytes (0 if not found).
 * Must be called before any PSRAM access.  Configures QPI mode, timing, and
 * enables memory-mapped writes.
 *
 * This function reconfigures the QSPI bus and must not execute from flash.
 */
size_t psram_init(void);

#endif /* PSRAM_H */
