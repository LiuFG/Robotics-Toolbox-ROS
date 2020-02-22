# Makefile for sample program
.PHONY			: all clean

# the program to build
NAME			:= GxAcquireContinuousOpenCV

# Build tools and flags
CXX			:= g++
LD			:= g++

OPENCV_INCLUDEPATH += -I/usr/local/include \
			-I/usr/local/include/opencv \
			-I/usr/local/include/opencv2

CPPFLAGS		:= -w -I$(GENICAM_ROOT_V2_3)/library/CPP/include \
			       -I$(GENICAM_ROOT_V2_3)/../../sdk/include \
				$(OPENCV_INCLUDEPATH)
ifeq ($(shell getconf LONG_BIT),64)
LDFLAGS			:=-lgxiapi -ldximageproc -lpthread \
                      -L$(GENICAM_ROOT_V2_3)/bin/Linux64_x64 \
                      -L$(GENICAM_ROOT_V2_3)/bin/Linux64_x64/GenApi/Generic \
                  -lGCBase_gcc40_v2_3 -lGenApi_gcc40_v2_3 -llog4cpp_gcc40_v2_3 -lLog_gcc40_v2_3 -lMathParser_gcc40_v2_3 \
			-lopencv_highgui -lopencv_core -lopencv_imgproc
else
LDFLAGS                 :=-lgxiapi ldximageproc -lpthread \
                      -L$(GENICAM_ROOT_V2_3)/bin/Linux32_i86 \
                      -L$(GENICAM_ROOT_V2_3)/bin/Linux32_i86/GenApi/Generic \
                  -lGCBase_gcc40_v2_3 -lGenApi_gcc40_v2_3 -llog4cpp_gcc40_v2_3 -lLog_gcc40_v2_3 -lMathParser_gcc40_v2_3 \
			-lopencv_highgui -lopencv_core -lopencv_imgproc
endif

all			: $(NAME)

$(NAME)			: $(NAME).o
	$(LD) -o $@ $^ $(CPPFLAGS) $(LDFLAGS)

$(NAME).o		: $(NAME).cpp
	$(CXX) $(CPPFLAGS) -c -o $@ $<

clean			:
	$(RM) $(NAME).o $(NAME)

