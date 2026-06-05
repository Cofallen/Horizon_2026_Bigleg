#include "ppo_actor.h"
#include "fc.h"
#include "math.h"

void ppo_actor_forward(float input[INPUT_DIM], float output[OUTPUT_DIM]){
    float h1[HIDDEN1];
    float h2[HIDDEN2];

    for (int i = 0; i < HIDDEN1; i++)
    {
        h1[i] = fc1_b[i];
        for (int j = 0; j < INPUT_DIM; j++)
        {
            h1[i] += fc1_w[i][j] * input[j];
        }
        h1[i] = tanhf(h1[i]);
    }
    
    for (int i = 0; i < HIDDEN2; i++)
    {
        h2[i] = fc2_b[i];
        for (int j = 0; j < HIDDEN1; j++)
        {
            h2[i] += fc2_w[i][j] * h1[j];
        }
        h2[i] = tanhf(h2[i]);
    }

    for (int i = 0; i < OUTPUT_DIM; i++)
    {
        output[i] = fc3_b[i];
        for (int j = 0; j < HIDDEN2; j++)
        {
            output[i] += fc3_w[i][j] * h2[j];
        }
    }
}