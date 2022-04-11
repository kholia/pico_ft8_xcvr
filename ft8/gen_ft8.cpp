#include "gen_ft8.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "pack.h"
#include "encode.h"
#include "constants.h"

void generate_ft8(char message[], uint8_t tone_sequence[])
{
    // int message_length = strlen(message);
    // First, pack the text data into binary message
    uint8_t packed[FTX_LDPC_K_BYTES];
    int rc = pack77(message, packed);

    if (rc < 0)
    {
        printf("Cannot parse message!\n");
        printf("RC = %d\n", rc);
    }

    printf("Packed data: ");
    for (int j = 0; j < 10; ++j)
    {
        printf("%02x ", packed[j]);
    }
    printf("\n");

    int num_tones = FT8_NN;

    // Second, encode the binary message as a sequence of FSK tones
    ft8_encode(packed, tone_sequence);

    printf("FSK tones: ");
    for (int j = 0; j < num_tones; ++j)
    {
        printf("%d", tone_sequence[j]);
    }
    printf("\n");

    return;
}
