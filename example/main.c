#define RSGLDEF 
#include "RSGL.h"

#include <stdio.h>

#define RPHYS_IMPLEMENTATION
#include "rphys.h"

void bodyCollideCallback(RPhys_body* body1, RPhys_body* body2) {
    printf("body %i colliding with %i\n", body1->index, body2->index);
}

int main() {
    RPhys_init();

    RPhys_body bodies[] = {
            {RPhys_shape_loadRect(RSGL_RECTF(200, 20, 20, 20), 0.90f, ICE), false},
            {RPhys_shape_loadPolygon(RSGL_RECTF(240, 20, 20, 20), 8, 9.0f, RUBBER), false},
            {RPhys_shape_loadRect(RSGL_RECTF(0, 200, 500, 100), 7.0f, CONCRETE_DRY), true},
            {RPhys_shape_loadRect(RSGL_RECTF(20, 400, 500, 100), 10.0f, CONCRETE_DRY), true}
    };

    RPhys_addBodies(bodies, sizeof(bodies) / sizeof(RPhys_body));

    RSGL_window* win = RSGL_createWindow("name", RSGL_RECT(0, 0, 700, 700), (u64)RSGL_CENTER);

    win->fpsCap = 60;

    u8 running = true;

    u8 player = 0;

    while (running) {
        while (RSGL_window_checkEvent(win)) {
            switch (win->event.type) {
                case RGFW_quit:
                    running = false;
                    break;
                case RGFW_keyReleased:
                    if (win->event.keyCode == RGFW_Space)
                        player++;
                    if (player >= 2)
                        player = 0;
                default:
                    break;
            }
        }

        if (RGFW_isPressedI(win, RGFW_Right))
            bodies[player].velocity.x += 0.005;
        else if (RGFW_isPressedI(win, RGFW_Left))
            bodies[player].velocity.x += -0.005;
        else 
            bodies[player].velocity.x = 0;

        if (RGFW_isPressedI(win, RGFW_Down))
            bodies[player].velocity.y = 5;
        if (RGFW_isPressedI(win, RGFW_Up))
            bodies[player].velocity.y = -2;    
        
        RPhys_run(bodyCollideCallback);

        RPhys_drawBodies();

        RSGL_window_clear(win, RSGL_RGB(0, 0, 0));
    }

    RSGL_window_close(win);
    RPhys_free();
}