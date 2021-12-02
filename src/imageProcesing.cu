#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>

__global__ void kernel()
{

}
void cudamain() 
{	
	kernel<<<1,1>>>();
	printf("Hola...%i\n",1);
	return;
}


__global__ void erosion16CudaKernel(unsigned short *imgOriginalCuda,unsigned short *imgTempCuda, int mask, int rows,int cols, int step)
{
	int r, c;
    bool dark;
	r = blockIdx.x;
	c = threadIdx.x;
    dark = false;
    //Get mask
    for (int tempi=r-mask; tempi<(r+mask);tempi++){
        if((tempi>0) && (tempi <rows)){
            for (int tempj=c-mask; tempj<(c+mask);tempj++){
              if((tempj>0) && (tempj <cols)){
                //If found a drak pixel enable fla
                if( imgOriginalCuda[(tempi * step) + (tempj * 1) ]== 0){
                  dark=true;
                }
              }else{
                dark=true;
              }
            }
        }else{
            dark=true;
        }
    }
        //If drak pixel eflag is nable then place dark pixel otherwise copy original image pixel
    if( dark){
        imgTempCuda[(r * step) + (c * 1) ] = 0;
    }
    else{
        imgTempCuda[(r * step) + (c * 1) ] = imgOriginalCuda[(r * step) + (c * 1) ];
    }
}
/*
__global__ void dilation16CudaKernel(cv::Mat &img,cv::Mat &imgTemp, int mask)
{
	printf("Starting...\n");

}
__global__ void edge16CudaKernel(cv::Mat &img,cv::Mat &imgEdge, int threshold16)
{
	printf("Starting...\n");

}

__global__ void erosion8CudaKernel(cv::Mat &img,cv::Mat &imgTemp, int mask)
{
	printf("Starting...\n");

}
__global__ void dilation8CudaKernel(cv::Mat &img,cv::Mat &imgTemp, int mask)
{
	printf("Starting...\n");

}


*/

void erosion16Cuda(cv::Mat &img,cv::Mat &imgTemp, int mask){
    cv::cuda::GpuMat gpuSOurce,gpuDst;
    gpuSOurce.upload(img);
    
    unsigned short *imgOriginalCuda;
    unsigned short *imgTempCuda;



    cudaMalloc((void **)&imgOriginalCuda,   gpuSOurce.rows*gpuSOurce.step);
    cudaMalloc((void **)&imgTempCuda,       gpuSOurce.rows*gpuSOurce.step);

    cudaMemcpyAsync(imgOriginalCuda, gpuSOurce.ptr<unsigned short>(), gpuSOurce.rows*gpuSOurce.step, cudaMemcpyDeviceToDevice);

	erosion16CudaKernel<<<gpuSOurce.rows, gpuSOurce.cols>>> (imgOriginalCuda, imgTempCuda,mask,gpuSOurce.rows,gpuSOurce.cols,gpuSOurce.step);

    cudaMemcpy(gpuSOurce.ptr<unsigned short>(), imgOriginalCuda, gpuSOurce.rows*gpuSOurce.step, cudaMemcpyDeviceToHost);

    // following code is just for testing and visualization...
    gpuSOurce.download(img);
    
}
/*
void dilation16Cuda		(cv::Mat &img,cv::Mat &imgTemp, int mask){

}
void edge16Cuda			(cv::Mat &img,cv::Mat &imgEdge, int threshold16){

}
void erosion8Cuda		(cv::Mat &img,cv::Mat &imgTemp, int mask){

}
void dilation8Cuda		(cv::Mat &img,cv::Mat &imgTemp, int mask){

}
*/