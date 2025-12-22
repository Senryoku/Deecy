/*
UP_RUMBLE - Version 1
"Everyone's favorite thinly veiled XInput wrapper!"

Define UP_RUMBLE_IMPLEMENTATION in at least *1* C/C++ source file

["my_epic_main.c"]
...
#define UP_RUMBLE_IMPLEMENTATION
...

Like that.

To use it, first call it:

...
set_gamepad_rumble(0, 0.5f, 0.5f, 2.0f);
...

Then on your main game loop, add this little function:

...
poll_gamepads(dt)
...

You must pass it the current delta time being spit out by your framework/engine/enginework.
By doing this it'll handle disabling rumble when needed.

You may then check if a gamepad is rumbling (maybe tumbling) by calling this:

...
if(is_gamepad_rumbling(2))
    ... do thing ...
...

One more thing, you must link to xinput!
[xinput1_4] specifically (as far as I tested.)

*/

#if !defined(UP_RUMBLE_H)

/* 
XInput supports max 4 gamepads.
Honestly that's a full party so I don't think you need more.
*/

#define UP_MAX_GAMEPADS 4

int set_gamepad_rumble(unsigned int gamepad, float left_rumble, float right_rumble, float duration);
int is_gamepad_rumbling(unsigned int gamepad);
void poll_gamepads(float dt);

#define UP_RUMBLE_H
#endif

#ifdef UP_RUMBLE_IMPLEMENTATION
#ifndef UP_RUMBLE_IMPLEMENTATION_ONCE
#define UP_RUMBLE_IMPLEMENTATION_ONCE

typedef struct _XINPUT_VIBRATION {
    unsigned wLeftMotorSpeed;
    unsigned wRightMotorSpeed;
} XINPUT_VIBRATION;
unsigned int XInputSetState(unsigned int            dwUserIndex, XINPUT_VIBRATION *pVibration);

float gamepad_timers[UP_MAX_GAMEPADS] = {0.0f, 0.0f, 0.0f, 0.0f};

int set_gamepad_rumble(unsigned int gamepad, float left_rumble, float right_rumble, float duration)
{
    if (gamepad > (UP_MAX_GAMEPADS - 1))
        return 0;
    XINPUT_VIBRATION state = {};
    state.wLeftMotorSpeed  = (unsigned)(left_rumble * 65535.0f);
    state.wRightMotorSpeed = (unsigned)(right_rumble * 65535.0f);

    if (XInputSetState(gamepad, &state) == 0){
        gamepad_timers[gamepad] = duration;
        return 1;
    }
    return 0;
}

int is_gamepad_rumbling(unsigned int gamepad)
{
    if (gamepad > (UP_MAX_GAMEPADS - 1))
        return 0;
    return gamepad_timers[gamepad] != 0.0f;
}

void poll_gamepads(float dt)
{
    for(int i = 0; i < UP_MAX_GAMEPADS; ++i){
        if(gamepad_timers[i] != 0.0f){
            gamepad_timers[i] -= dt;
            if(gamepad_timers[i] <= 0.0f){
                set_gamepad_rumble(i, 0.0f, 0.0f, 0.0f);
            }
        }
    }
}

#endif
#endif