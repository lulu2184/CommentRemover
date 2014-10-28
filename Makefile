remover : main.o remover.o state.o processor.o factory.o
	g++ -g -o remover main.o remover.o state.o processor.o factory.o
main.o : main.cpp processor.h factory.h
	g++ -g -c main.cpp
remover.o : remover.cpp state.h CommentRemover.h
	g++ -g -c remover.cpp
state.o : state.cpp state.h
	g++ -g -c state.cpp
processor.o : processor.cpp processor.h CommentRemover.h factory.h
	g++ -g -c processor.cpp
factory.o : factory.cpp factory.h processor.h
	g++ -g -c factory.cpp 

clean:
	rm *.o remover