# Localization Challenge
# Makefile

INCLUDE = -I/usr/include/
LIBRARIES = controller.a -L/usr/lib/x86_64-linux-gnu/ -lGL -lGLU -lglut -lm -lXi -lXmu

COMPILER = g++ --std=c++11
COMPILERFLAGS = -no-pie $(INCLUDE)


PROGRAM =	localization_test

SOURCE =	main.cpp helper_func.cpp
HEADERS =	main.h helper_func.h

OBJECT =	main.o helper_func.o


.cc.o: $(SOURCE)
	$(COMPILER) -c $(COMPILERFLAGS) $<

all: $(PROGRAM)

$(PROGRAM): $(OBJECT) $(SOURCE)
	$(COMPILER) $(COMPILERFLAGS) -o $(PROGRAM) $(OBJECT) $(LIBRARIES)

clean:
	-rm -rf core *.o *~ .*~ $(PROGRAM)
