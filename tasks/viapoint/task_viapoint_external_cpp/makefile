TARGET = task_viapoint_external_cpp
FILEIO_DIR = ../../../fileio/cpp/

all: ${TARGET}


${TARGET}: ${TARGET}.o
	make -C ${FILEIO_DIR}
	g++ $< -L${FILEIO_DIR} -lfileio -o $@

clean:
	rm -f *.o
	
clobber: clean
	rm ${TARGET}

%.o: %.cpp
	g++  -I${FILEIO_DIR} -c $< -o $@
	
	
