# Copyright 2013 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT         := Client
SRC				:= .
#/Users/Kevin/CubicHand/mbed/projects/Client/ Users/Kevin/CubicHand/mbed/libs/cc3000_hostdriver_mbedsocket
#SRC				+= ./../libs/cc3000_hostdriver_mbedsocket
DEVICES         := KL25Z
GCC4MBED_DIR    := /Users/Kevin/Documents/adamgreen-gcc4mbed-1f1ebc4
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 1
INCDIRS			:= /Users/Kevin/CubicHand/mbed/libs
INCDIRS			+= /Users/Kevin/CubicHand/mbed/libs/cc3000_hostdriver_mbedsocket
#LIBS_SUFFIX		:= /Users/Kevin/CubicHand/mbed/libs/cc3000_hostdriver_mbedsocket/KL25Z/*.o 

include $(GCC4MBED_DIR)/build/gcc4mbed.mk
include /Users/Kevin/CubicHand/mbed/libs/cc3000_hostdriver_mbedsocket/Makefile
