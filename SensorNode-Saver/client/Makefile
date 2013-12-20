HEADER_H=../../TmoteComm.h
BUILD_EXTRA_DEPS += OutputSaver.class

OutputSaver.class: $(wildcard *.java) MatricesArgs.java
	javac *.java

MatricesArgs.java: $(HEADER_H)
	mig -target=null -java-classname=MatricesArgs java $(HEADER_H) matrices_args_msg -o $@

clean:
	rm -f *.class $(GEN)