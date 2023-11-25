#define RSGLDEF 
#include "RSGL.h"

#include <stdio.h>

#define RPHYS_IMPLEMENTATION
#include "rphys.h"

int main() {
    RPhys_init();

    RPhys_body bodies[] = {
            {RPhys_shape_loadRect(RSGL_RECT(200, 20, 20, 20), 0.90f), false},
            {RPhys_shape_loadPolygon(RSGL_RECT(240, 20, 20, 20), 8, 9.0f), false},
            {RPhys_shape_loadRect(RSGL_RECT(0, 200, 500, 100), 7.0f), true},
            {RPhys_shape_loadRect(RSGL_RECT(0, 400, 500, 100), 10.0f), true}
    };

    RPhys_addBodies(bodies, sizeof(bodies) / sizeof(RPhys_body));

    RSGL_window* win = RSGL_createWindow("name", RSGL_RECT(0, 0, 500, 700), (u64)RSGL_CENTER);

    win->fpsCap = 60;

    u8 running = true;

    u32 font = RSGL_loadFont("DejaVuSansMono.ttf");
    RSGL_setFont(font);

    while (running) {
        while (RSGL_window_checkEvent(win)) {
            switch (win->event.type) {
                case RGFW_quit:
                    running = false;
                    break;
                default:
                    break;
            }
        }
        
        if (RGFW_isPressedI(win, RGFW_Right))
            bodies[0].velocity.x += 0.005;
        else if (RGFW_isPressedI(win, RGFW_Left))
            bodies[0].velocity.x += -0.005;
        else 
            bodies[0].velocity.x = 0;

        if (RGFW_isPressedI(win, RGFW_Down))
            bodies[0].velocity.y = 5;
        if (RGFW_isPressedI(win, RGFW_Up))
            bodies[0].velocity.y = -2;    
        
        RPhys_run();

        RPhys_drawBodies();

        RSGL_window_clear(win, RSGL_RGB(0, 0, 0));
    }

    RSGL_window_close(win);
    RPhys_free();
}