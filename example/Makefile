ifeq '$(findstring ;,$(PATH))' ';'
    detected_OS := Windows
else
    detected_OS := $(shell uname 2>/dev/null || echo Unknown)
    detected_OS := $(patsubst CYGWIN%,Cygwin,$(detected_OS))
    detected_OS := $(patsubst MSYS%,MSYS,$(detected_OS))
    detected_OS := $(patsubst MINGW%,MSYS,$(detected_OS))
endif

ifeq ($(detected_OS),Windows)
	LIBS := -lopengl32 -lshell32 -lgdi32 -lm
endif
ifeq ($(detected_OS),Darwin)        # Mac OS X
	LIBS := -lm -framework Foundation -framework AppKit -framework OpenGL -framework CoreVideo
endif
ifeq ($(detected_OS),Linux)
    LIBS := -I./include -lX11 -lGLX -lm
endif

OUTPUT = rphys

all:
	make RSGL.o
	gcc main.c -O3 -I../ -I./include $(LIBS) RSGL.o -o $(OUTPUT) 

debug:
	make RSGL.o
	gcc main.c -lGL -g -I../ -I./include $(LIBS) RSGL.o -o $(OUTPUT)
	./$(OUTPUT)

RSGL.o:
	cp include/RSGL.h RSGL.c
	gcc -c RSGL.c -D RSGL_IMPLEMENTATION -D RGFW_NO_JOYSTICK_CODES -I./include/
	rm RSGL.c


clean:
	rm $(OUTPUT)