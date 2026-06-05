#ifndef __PPO_ACTOR_H
#define __PPO_ACTOR_H

#define INPUT_DIM 4
#define HIDDEN1 256
#define HIDDEN2 256
#define OUTPUT_DIM 1

void ppo_actor_forward(float input[INPUT_DIM], float output[OUTPUT_DIM]);

#endif