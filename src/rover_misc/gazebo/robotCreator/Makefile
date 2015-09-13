CC=g++

LIBS=-lboost_system -lboost_filesystem

robotCreator: robotCreator.o
	$(CC) -o $@ $^ $(LIBS)

.PHONY: clean

clean:
	rm -f *.o *~
