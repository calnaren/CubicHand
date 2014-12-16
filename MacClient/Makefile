all: Client

Client: main.o TCPClient.o 
	g++ main.o TCPClient.o -o Client

main.o: main.cpp
	g++ -c main.cpp

TCPClient.o: TCPClient.cpp
	g++ -c TCPClient.cpp

clean:
	rm -rf *o Client

