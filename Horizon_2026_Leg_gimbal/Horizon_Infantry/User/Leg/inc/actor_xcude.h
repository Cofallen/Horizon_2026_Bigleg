#ifndef __ACTOR_XCUDE_H__
#define __ACTOR_XCUDE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "network.h"
#include "network_data.h"

typedef struct
{
    ai_handle handle;

    ai_buffer* input;
    ai_buffer* output;

    ai_u8* activations;

    float* input_data;
    float* output_data;

    uint16_t input_size;
    uint16_t output_size;

    uint8_t initialized;

} AIActor;


void AIActor_Init(AIActor* actor);

void AIActor_Run(
    AIActor* actor,
    const float* obs
);

float AIActor_GetOutput(
    AIActor* actor,
    uint16_t index
);

#ifdef __cplusplus
}
#endif


#endif /* __ACTOR_XCUDE_H__ */
