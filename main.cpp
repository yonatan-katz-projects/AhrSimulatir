#include <iostream>
#include "Strategy.h"
#include "AhrSimCore.h"
#include "FreeGLUTCallbacks.h"

using namespace std;

int main(int argc, char** argv)
{		
	ToyStrategy strategy;
	
	AhrSimCore simulator(&strategy);			
	
	return glutmain(argc, argv, 1024, 768, "AHR game simulator", &simulator);
}

