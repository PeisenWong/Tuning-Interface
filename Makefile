CC = "g++"
PROJECT = tune
SRC = main.cpp

LIBS = `pkg-config opencv4 --cflags --libs`

$(PROJECT) : $(SRC)
	$(CC) $(SRC) $(LIBS) ./serialLib/serialib.cpp -lbluetooth -lX11 -I./inc -o $(PROJECT) 