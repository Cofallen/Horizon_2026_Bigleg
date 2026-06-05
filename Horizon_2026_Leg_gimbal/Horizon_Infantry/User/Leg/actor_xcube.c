#include "actor_xcude.h"

#include <string.h>


static AI_ALIGNED(4)
ai_u8 g_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

static float g_input_data[AI_NETWORK_IN_1_SIZE];

static float g_output_data[AI_NETWORK_OUT_1_SIZE];


void AIActor_Init(AIActor* actor)
{
    ai_error err;

    memset(actor, 0, sizeof(AIActor));

    actor->activations = g_activations;

    actor->input_data = g_input_data;
    actor->output_data = g_output_data;

    actor->input_size = AI_NETWORK_IN_1_SIZE;
    actor->output_size = AI_NETWORK_OUT_1_SIZE;

    const ai_handle acts[] =
    {
        actor->activations
    };

    err = ai_network_create_and_init(
        &actor->handle,
        acts,
        NULL
    );

    if(err.type != AI_ERROR_NONE)
    {
        while(1);
    }

    actor->input = ai_network_inputs_get(
        actor->handle,
        NULL
    );

    actor->output = ai_network_outputs_get(
        actor->handle,
        NULL
    );

    actor->input[0].data =
        AI_HANDLE_PTR(actor->input_data);

    actor->output[0].data =
        AI_HANDLE_PTR(actor->output_data);

    actor->initialized = 1;
}


void AIActor_Run(
    AIActor* actor,
    const float* obs
)
{
    if(actor->initialized == 0)
    {
        return;
    }

    memcpy(
        actor->input_data,
        obs,
        actor->input_size * sizeof(float)
    );

    ai_network_run(
        actor->handle,
        actor->input,
        actor->output
    );
}


float AIActor_GetOutput(
    AIActor* actor,
    uint16_t index
)
{
    if(index >= actor->output_size)
    {
        return 0.0f;
    }

    return actor->output_data[index];
}