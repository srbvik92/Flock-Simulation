CC          = g++ 
CFLAGS = -g -Wall -Wextra -Wpedantic -pedantic-errors

#-----------------------------------------
#Optimization ----------------------------
OPT   = -O3 -Wno-deprecated

#GL_LIB = -framework OpenGL -framework GLUT -framework foundation
GL_LIB = -lGL -lglut -lGLU

#-----------------------------------------

TARGETS = fishtank viewer
OBJECTS = Boids.o kdTree.o

#-----------------------------------------

LIBS = -L/usr/lib64 -lblas
INCS = -I../common -I/usr/local/include/eigen3 -I/usr/include/eigen3 -I/afs/umbc.edu/users/a/d/adamb/pub/

CCOPTS = $(OPT) $(INCS) 
LDOPTS = $(OPT) $(INCS) $(LIBS) 

#-----------------------------------------
#-----------------------------------------

.PHONY: run clean test

default: $(TARGETS)

clean:
	/bin/rm -f *.o $(TARGETS)

run:
	make && ./fishtank sample.in

#-----------------------------------------
#-----------------------------------------

viewer: viewer.cpp
	$(CC) $(INCS) $(LDOPTS) $(GL_LIB) $< -o $@

fishtank: fishtank.o $(OBJECTS)
	$(CC) $(CFLAGS) $(INCS) $(LDOPTS) $^ -o $@

fishtank.o: fishtank.cpp
	$(CC) $(CFLAGS) $(LDOPTS) -c $< -o $@

Boids.o: Boids.cpp Boids.h
	$(CC) $(CFLAGS) $(LDOPTS) -c $< -o $@

