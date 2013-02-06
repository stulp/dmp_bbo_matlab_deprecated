all: libfileio.a

clean:
	rm -f *.o *~
	
clobber: clean
	rm *.a

lib%.a: %.o
	ar rcs $@ $<

%.o: %.cpp
	g++ -c $< -o $@
	
	
	
