/*
* Copyright (c) 2021-23 ColleagueRiley ColleagueRiley@gmail.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following r estrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*
*
*

/*
    define args
    (MAKE SURE RPHYS_IMPLEMENTATION is in at least one header or you use -DRSGL_IMPLEMENTATION)
	#define RPHYS_IMPLEMENTATION - makes it so source code is include
*/

#ifndef RPHYS_H
#define RPHYS_H

#ifndef RPHYSDEF
#ifdef __APPLE__
#define RPHYSDEF extern inline
#else
#define RPHYSDEF inline
#endif
#endif

#ifndef RPHYS_BODIES_INIT
#define RPHYS_BODIES_INIT 64
#endif

#ifndef RPHYS_BODIES_NEW
#define RPHYS_BODIES_NEW 5
#endif

#ifndef RPHYS_MAX_VERTICES
#define RPHYS_MAX_VERTICES 24
#endif

#ifndef RPHYS_MALLOC
#include <memory.h>

#define RPHYS_MALLOC malloc
#define RPHYS_REALLOC realloc
#define RPHYS_FREE free
#endif

#ifdef RSGL_H
/* for loading polygons */
#include <math.h>

#ifndef PI
    #define PI 3.14159265358979323846f
#endif

#ifndef DEG2RAD
    #define DEG2RAD (PI/180.0f)
#endif
#endif

#ifdef RSGL_H
typedef RSGL_pointF vector2;
#else

#if !defined(u8)
    #include <stdint.h>

    typedef uint8_t     u8;
	typedef int8_t      i8;
	typedef uint16_t   u16;
	typedef int16_t    i16;
	typedef uint32_t   u32;
	typedef int32_t    i32;
	typedef uint64_t   u64;
	typedef int64_t    i64;
#endif

#ifndef RPHS_VECTOR_DEFINED
typedef struct vector2 {float x, y;};
#endif
#endif

typedef struct RPhys_circle {vector2 v; float d;} RPhys_circle;
typedef struct RPhys_rect {vector2 v; float w, h;} RPhys_rect;

/* add two 2D vectors */
RPHYSDEF vector2 RPhys_addVector2(vector2 v1, vector2 v2);
RPHYSDEF vector2 RPhys_subtractVector2(vector2 v1, vector2 v2);
RPHYSDEF vector2 RPhys_multiplyVector2(vector2 v1, vector2 v2);
RPHYSDEF vector2 RPhys_divideVector2(vector2 v1, vector2 v2);

/* init RPhys (allocations, ect)*/
RPHYSDEF void RPhys_init(void);
/* do physics processing for all the bodies */
void RPhys_run(void);
/* free and deinit RPhys data (for you're done using it) */
RPHYSDEF void RPhys_free(void);
/* set current gravity (9.8 m/s^2 by default)*/
RPHYSDEF void RPhys_setGravity(vector2 gravity);
/* set current air density (1.29 kg m−3 default) */
RPHYSDEF void RPhys_setAirDensity(float density);

typedef enum RPhys_shape_option {
    RPHYS_CIRCLE, /* so we don't have to handle all 360 points */
    RPHYS_RECT, /* rectangle shape */
    RPHYS_RECT_POLYGON, /* polygon with rectangle data */
    RPHYS_POLYGON /* everything else that's not a circle or have a rectangle shape */
} RPhys_shape_option;

typedef struct RPhys_shape {
    RPhys_shape_option s; /* actual shape */
    float mass; /* mass of the body / shape (in grams) */

    union {
        RPhys_circle c;
        RPhys_rect r;
        vector2 vertices[RPHYS_MAX_VERTICES];
    };

    size_t vertexCount; /* how many vertices are in the vertices array (if the shape is a polygon) */
} RPhys_shape;

RPHYSDEF bool RPhys_circleCollideRect(RPhys_circle c, RPhys_rect rect);
RPHYSDEF bool RPhys_circleCollide(RPhys_circle cir1, RPhys_circle cir2);
RPHYSDEF bool RPhys_rectCollide(RPhys_rect rect, RPhys_rect rect2);
RPHYSDEF bool RPhys_shapeCollide(RPhys_shape s, RPhys_shape s2);

RPHYSDEF RPhys_rect RPhys_triangleToRect(vector2* triangle);

#ifdef RSGL_H
/* creates an RPhys_shape from an RSGL_rect structure */
RPHYSDEF RPhys_shape RPhys_shape_loadRect(RSGL_rect r, float mass);
/* creates an polygon RPhys_shape using an RSGL_rect structure and a given number of sides */
RPHYSDEF RPhys_shape RPhys_shape_loadPolygon(RSGL_rect r, u32 sides, float mass);
/* creates an RPhys_shape from an RSGL_circle structure */
RPHYSDEF RPhys_shape RPhys_shape_loadCircle(RSGL_circle c, float mass);
/* creates an RPhys_shape from an RSGL_triangle structure */
RPHYSDEF RPhys_shape RPhys_shape_loadTriangle(RSGL_triangle t, float mass);
/* creates an RPhys_shape from an RSGL_point structure */
RPHYSDEF RPhys_shape RPhys_shape_loadPoint(RSGL_point p, float mass);

/* creates an RSGL_rect from rectangular polygon data of a RPhys_shape */
RPHYSDEF RSGL_rect RPhys_shape_getRect(RPhys_shape shape);
/* 
    creates an RSGL_rect from the polygon data of a RPhys_shape 
    (this assumes any type of polygon so this will be slower than using `RPhys_shape_getRect`)
*/
RPHYSDEF RSGL_rect RPhys_shape_getPolyRect(RPhys_shape shape);
/* creates an RSGL_circle from the RPhys_circle structure of a RPhys_shape */
RPHYSDEF RSGL_circle RPhys_shape_getCircle(RPhys_shape shape);
/* creates an RSGL_triangle from triangular polygon data of a RPhys_shape */
RPHYSDEF RSGL_triangle RPhys_shape_getTriangle(RPhys_shape shape);
/* creates an RSGL_point from a single point of the data from RPhys_shape */
RPHYSDEF RSGL_point RPhys_shape_getPoint(RPhys_shape shape);
#endif

typedef struct RPhys_body {
    RPhys_shape shape; /* shape of object */
    bool floating; /* not affected by gravity */

    vector2 velocity; /* speed of body (in px/s) */
    vector2 force; /* force on body (in newtons) */
    
    u16 index; /* index of body (in array) */
} RPhys_body;

/* get current acceleration of an object (based on force and mass) (in px / s ^ 2 (pixels per second per second)) */
RPHYSDEF vector2 RPhys_body_getAcceleration(RPhys_body* body);

/* add new body to body array */
RPHYSDEF void RPhys_addBody(RPhys_body* body);
/* add multiple bodies to the body array */
RPHYSDEF void RPhys_addBodies(RPhys_body* bodies, size_t count);
/* remove body from body array */
RPHYSDEF void RPhys_removeBody(RPhys_body* body);

#ifdef RSGL_H
/* draw the bodies in the body array (guesses shape of polygon) */
RPHYSDEF void RPhys_drawBodies(void);
#endif

#endif /* ndef RPHYS_H */

#ifdef RPHYS_IMPLEMENTATION

vector2 RPhys_gravity = {0.0f, 9.8f};
float RPhys_airDensity = 1.29f;

size_t RPhys_len, RPhys_cap;
RPhys_body** RPhys_bodies;

vector2 RPhys_addVector2(vector2 v1, vector2 v2) {
    return (vector2){v1.x + v2.x, v1.y + v2.y};
}

vector2 RPhys_subtractVector2(vector2 v1, vector2 v2) {
    return (vector2){v1.x - v2.x, v1.y - v2.y};
}

vector2 RPhys_multiplyVector2(vector2 v1, vector2 v2) {
    return (vector2){v1.x * v2.x, v1.y * v2.y};
}

vector2 RPhys_divideVector2(vector2 v1, vector2 v2) {
    return (vector2){v1.x / v2.x, v1.y / v2.y};
}

void RPhys_init(void) {
    RPhys_bodies = (RPhys_body**)malloc(sizeof(RPhys_body**) * RPHYS_BODIES_INIT);
    
    RPhys_len = 0;
    RPhys_cap = RPHYS_BODIES_INIT;
}

void RPhys_run(void) {
    static time_t startTime = 0;
    if (startTime == 0)
        startTime = clock();

    size_t i;
    for (i = 0; i < RPhys_len; i++) {
        RPhys_body* body = RPhys_bodies[i];

        vector2 grav = RPhys_multiplyVector2(RPhys_gravity, (vector2){body->shape.mass, body->shape.mass});

        if (body->floating == true)
            grav = (vector2){0, 0};

        body->force = RPhys_addVector2(body->force, grav);

        time_t t = clock() - startTime;

        size_t i2; 

        bool collide = false;

        for (i2 = 0; i2 < RPhys_len; i2++) {
            RPhys_body* body2 = RPhys_bodies[i2];

            if (body->index == body2->index || body->floating)
                continue;
            
            if (RPhys_shapeCollide(body->shape, body2->shape)) {
                body->force = RPhys_subtractVector2(body->force, grav);
                
                vector2 grav2 = RPhys_multiplyVector2(RPhys_gravity, (vector2){body->shape.mass - body2->shape.mass, body->shape.mass - body2->shape.mass});
                collide = true;
                if (grav2.x < 0.0f || grav2.y < 0.0f)
                    continue;
                
                body2->force = RPhys_addVector2(body2->force, grav2);
            }
        }
        
        vector2 acceleration = RPhys_body_getAcceleration(body);
        RPhys_multiplyVector2(acceleration, (vector2){t, t});

        if (collide == false)
            body->velocity = RPhys_addVector2(body->velocity, acceleration);

        /* lazy way to do air resistance */
        if (body->floating == false)
            body->velocity.y = ((RPhys_airDensity * body->shape.mass) / (body->velocity.y)) * 10;
        
        if (body->shape.s == RPHYS_RECT || body->shape.s == RPHYS_RECT_POLYGON)
            body->shape.r.v = RPhys_addVector2(body->shape.c.v, body->velocity);
        else if (body->shape.s == RPHYS_POLYGON) {
            size_t v;
            for (v = 0; v < body->shape.vertexCount; v++)
                body->shape.vertices[v] = RPhys_addVector2(body->shape.vertices[v], body->velocity);
        }
        else 
            body->shape.c.v = RPhys_addVector2(body->shape.c.v, body->velocity);
    
        if (collide == false)
            body->force = RPhys_subtractVector2(body->force, grav);
    }
    
    startTime = clock();
}

void RPhys_free(void) {
    free(RPhys_bodies);
}

void RPhys_setGravity(vector2 gravity) {
    RPhys_gravity = gravity;
}

void RPhys_setAirDensity(float density) {
    RPhys_airDensity = density;
}

bool RPhys_circleCollide(RPhys_circle cir, RPhys_circle cir2) {
    float distanceBetweenCircles = (float) sqrtf(
        (cir2.v.x - cir.v.x) * (cir2.v.x - cir.v.x) + 
        (cir2.v.y - cir.v.y) * (cir2.v.y - cir.v.y)
    );

    return !(distanceBetweenCircles > (cir.d/2) + (cir2.d/2)); /* check if there is a collide */
}

bool RPhys_rectCollide(RPhys_rect r, RPhys_rect r2) { 
    return (r.v.x + r.w >= r2.v.x && r.v.x <= r2.v.x + r2.w && r.v.y + r.h >= r2.v.y && r.v.y <= r2.v.y + r2.h); 
}

bool RPhys_circleCollideRect(RPhys_circle c, RPhys_rect r) {
    /* test cords */
    float testX = c.v.x; 
    float testY = c.v.y;

    /* fill cords based on x/ys of the shapes */
    if (c.v.x < r.v.x)
      testX = r.v.x;

    else if (c.v.x > r.v.x + r.w) 
      testX = r.v.x - r.w;

    if (c.v.y < r.v.x)  
      testY = r.v.x;  

    else if (c.v.y > r.v.y + r.h)
      testY = r.v.y + r.h; 
    
    /* check */
    return ( sqrtf( ( (c.v.x - testX) * (c.v.x - testX) ) + ( (c.v.y - testY) * (c.v.y - testY) ) )  <= (c.d/2) );
}

RPhys_rect RPhys_triangleToRect(vector2* triangle) {
    RPhys_rect rect = {-10.1f, -10.1f, 0.0f, 0.0f};
    
    u8 i;
    for (i = 0; i < 3; i++) {
        if (triangle[i].x < rect.v.x)
            rect.v.x = triangle[i].x;
        if (triangle[i].y < rect.v.y)
            rect.v.y = triangle[i].y;
        if (triangle[i].x > rect.w)
            rect.w = triangle[i].x;
        if (triangle[i].y > rect.h)
            rect.h = triangle[i].y;
    }

    return rect;
}

bool RPhys_shapeCollide(RPhys_shape s, RPhys_shape s2) {
    if (s.vertexCount == 3) {
        RPhys_rect r1 = RPhys_triangleToRect(s.vertices);

        if (s2.vertexCount == 3)
            return RPhys_rectCollide(r1, RPhys_triangleToRect(s2.vertices));

        if (s2.s == RPHYS_CIRCLE)
            return RPhys_circleCollideRect(s2.c, r1);
    
        if (s2.s == RPHYS_RECT || s2.s == RPHYS_RECT_POLYGON)
            return RPhys_rectCollide(r1, s2.r);
    }

    if ((s.s == RPHYS_RECT || s.s == RPHYS_RECT_POLYGON) && 
        (s2.s == RPHYS_RECT || s2.s == RPHYS_RECT_POLYGON))
            return RPhys_rectCollide(s.r, s2.r);
    if ((s.s == RPHYS_RECT || s.s == RPHYS_RECT_POLYGON) && s2.s == RPHYS_CIRCLE)
        return RPhys_circleCollideRect(s2.c, s.r);
    if ((s2.s == RPHYS_RECT || s2.s == RPHYS_RECT_POLYGON) && s.s == RPHYS_CIRCLE)
       return RPhys_circleCollideRect(s.c, s2.r);
    if (s.s == RPHYS_CIRCLE && s2.s == RPHYS_CIRCLE)
        return RPhys_circleCollide(s.c, s2.c);
}

#ifdef RSGL_H
RPhys_shape RPhys_shape_loadRect(RSGL_rect r, float mass) {
    RPhys_shape s = {RPHYS_RECT, mass, {}, 0};
    s.r = (RPhys_rect){(vector2){(float)r.x, (float)r.y}, (float)r.w, (float)r.h};

    return s;
}

RPhys_shape RPhys_shape_loadPolygon(RSGL_rect r, u32 sides, float mass) {
    RPhys_shape s = {RPHYS_RECT_POLYGON, mass, {}, sides};
    s.r = (RPhys_rect){(vector2){r.x, r.y}, r.w, r.h};

    return s;
}

RPhys_shape RPhys_shape_loadCircle(RSGL_circle c, float mass) {
    RPhys_shape s = {RPHYS_CIRCLE, mass, {}, 0};
    s.c = (RPhys_circle){(vector2){c.x, c.y}, c.d};

    return s;
}

RPhys_shape RPhys_shape_loadTriangle(RSGL_triangle t, float mass) {
    RPhys_shape s = {RPHYS_POLYGON, mass, {}, 3};
    s.vertices[0] = (vector2){t.p1.x, t.p1.y};
    s.vertices[1] = (vector2){t.p2.x, t.p2.y};
    s.vertices[2] = (vector2){t.p3.x, t.p3.y};

    return s;
}

RPhys_shape RPhys_shape_loadPoint(RSGL_point p, float mass) {
    RPhys_shape s = {RPHYS_RECT, mass, {}, 1};
    s.r = (RPhys_rect){(vector2){p.x, p.y}, 1, 1};

    return s;
}

RSGL_rect RPhys_shape_getRect(RPhys_shape shape) {
    if (shape.s == RPHYS_CIRCLE)
        return (RSGL_rect){shape.c.v.x, shape.c.v.y, shape.c.d, shape.c.d};
    if (shape.s == RPHYS_RECT)
        return (RSGL_rect){shape.r.v.x, shape.r.v.y, shape.r.w, shape.r.h};
    if (shape.vertexCount < 4)
        return (RSGL_rect){0, 0, 0, 0};

    RSGL_point p = RPhys_shape_getPoint(shape);
    return RSGL_RECT(   p.x, p.y, 
                        (shape.vertices[2].x - p.x), 
                        (shape.vertices[2].y - p.y)
                    );
}

RSGL_rect RPhys_shape_getPolyRect(RPhys_shape shape) {
    if (shape.s == RPHYS_CIRCLE)
        return (RSGL_rect){shape.c.v.x, shape.c.v.y, shape.c.d, shape.c.d};
    if (shape.s == RPHYS_RECT_POLYGON)
        return (RSGL_rect){shape.r.v.x, shape.r.v.y, shape.r.w, shape.r.h};
    
    RSGL_point p = {-500, -500};
    RSGL_area a = {0, 0};

    size_t i;
    for (i = 0; i < shape.vertexCount; i++) {
        if ((shape.vertices[i].x) < p.x || p.x == -500)
            p.x = (shape.vertices[i].x);
        
        if ((shape.vertices[i].y) < p.y || p.y == -500)
            p.y = shape.vertices[i].y;

        if ((shape.vertices[i].x) > a.w)
            a.w = shape.vertices[i].x;
        
        if ((shape.vertices[i].y) > a.h)
            a.h = shape.vertices[i].y;
    }

    return (RSGL_rect){p.x, p.y, a.w - p.x, a.h - p.x};
}

RSGL_circle RPhys_shape_getCircle(RPhys_shape shape) {
    if (shape.s == RPHYS_RECT)
        return RSGL_CIRCLE(shape.r.v.x, shape.r.v.y, shape.r.w);
    if (shape.s != RPHYS_CIRCLE)
        return RSGL_CIRCLE(0, 0, 0);
    return RSGL_CIRCLE(shape.c.v.x, shape.c.v.y, shape.c.d);
}

RSGL_triangle RPhys_shape_getTriangle(RPhys_shape shape) {
    if (shape.s == RPHYS_CIRCLE || shape.vertexCount < 3)
        return (RSGL_triangle){};
    return (RSGL_triangle){
                            (RSGL_point){shape.vertices[0].x, shape.vertices[0].y},
                            (RSGL_point){shape.vertices[1].x, shape.vertices[1].y},
                            (RSGL_point){shape.vertices[2].x, shape.vertices[2].y}
                          };
}

RSGL_point RPhys_shape_getPoint(RPhys_shape shape) {
    if (shape.s == RPHYS_CIRCLE)
        return (RSGL_point){shape.c.v.x, shape.c.v.y};
    
    if (shape.vertexCount < 1)
        return (RSGL_point){0, 0};
    
    return (RSGL_point){shape.vertices[0].x, shape.vertices[0].y};
}
#endif

vector2 RPhys_body_getAcceleration(RPhys_body* body) {
    /* a = (fnet / mass) */
    return (vector2){(body->force.x / body->shape.mass), (body->force.y / body->shape.mass)};
}

void RPhys_addBody(RPhys_body* body) {
    if (RPhys_len >= RPhys_cap) {
        RPhys_bodies = (RPhys_body**)realloc(RPhys_bodies, sizeof(RPhys_body**) * (RPHYS_BODIES_NEW + RPhys_len));
        RPhys_len += RPHYS_BODIES_NEW;
    }

    body->index = RPhys_len;

    body->velocity = (vector2){0, 0};
    body->force = (vector2){0, 0};

    RPhys_bodies[body->index] = body;

    RPhys_len += 1; 
}

void RPhys_addBodies(RPhys_body* bodies, size_t count) {
    size_t i;
    for (i = 0; i < count; i++)
        RPhys_addBody(&bodies[i]);
}

void RPhys_removeBody(RPhys_body* body) {
    RPhys_len -= 1;
    memcpy(RPhys_bodies + body->index, RPhys_bodies + body->index + 1, (RPhys_len - body->index) * sizeof(RPhys_body**));
}

#ifdef RSGL_H
void RPhys_drawBodies(void) {
    size_t i;
    for (i = 0; i < RPhys_len; i++) {
        RPhys_shape shape = RPhys_bodies[i]->shape;
        
        if (shape.s == RPHYS_CIRCLE) {
            RSGL_drawCircle(RPhys_shape_getCircle(shape), RSGL_RGB(255, 0 ,0));
            continue;
        }


        if (shape.vertexCount == 4 || shape.s == RPHYS_RECT) {
            RSGL_drawRect(RPhys_shape_getRect(shape), RSGL_RGB(255, 0, 0));
            continue;         
        }

        if (shape.vertexCount == 1) {
            RSGL_drawPoint(RPhys_shape_getPoint(shape), RSGL_RGB(255, 0, 0));
            continue;
        }

        if (shape.vertexCount == 3) {
            RSGL_drawTriangle(RPhys_shape_getTriangle(shape), RSGL_RGB(255, 0, 0));
            continue;         
        }

        RSGL_rect r = RPhys_shape_getPolyRect(shape);
        RSGL_drawPolygon(r, (shape.s == RPHYS_RECT_POLYGON) ? shape.vertexCount :shape.vertexCount / 3, RSGL_RGB(255, 0, 0));
    }
}
#endif

#endif /* RPHYS_IMPLEMENTATION */