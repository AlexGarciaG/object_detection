#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include "ros/ros.h"
#include "utils.h"

__global__ void kernel()
{
		printf("Starting...\n");

}

void cudamain() 
{	
	kernel<<<1,1>>>();
	printf("Hola...%i\n",1);
	return;
}
