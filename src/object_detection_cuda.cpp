#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Dynamic configure
#include <dynamic_reconfigure/server.h>
#include <object_detection/object_detectionConfig.h>
//Threding
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include "utils.h"

void cudamain();
void erosion16Cuda(cv::Mat &img,cv::Mat &imgTemp, int mask);
using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";




class ImageConverter
{
  //ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  //Images 
    //Original
  cv::Mat image;
    //COnverted
  cv::Mat imageEdge;

  const int maskR=1;
  const int maskSize=1+(2*maskR);
  //Ros for dynamic reconfigure
  dynamic_reconfigure::Server<object_detection::object_detectionConfig> server;
  dynamic_reconfigure::Server<object_detection::object_detectionConfig>::CallbackType f;


public:
    int vardilation16=0;
    int varerosion16=2;
    int varopening16=0;
    int varclosing16=0;
    int varThreshold16=1000;
    //8 bits
    int vardilation8=0;
    int varerosion8=1;
    int varopening8=0;
    int varclosing8=0;
        long double msdilation16=0;
    long double mserosion16=2;
    long double msopening16=0;
    long double msclosing16=0;
    long double msThreshold16=1000;
    long double ms16=1000;
    //8 bits
    long double msdilation8=0;
    long double mserosion8=1;
    long double msopening8=0;
    long double msclosing8=0;
    long double msCounter=0;
    long double ms8=1000;
  ImageConverter(): it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &ImageConverter::imageCb, this);
    //Publisher laser scan
    //image_pub_ = it_.advertise("/camera/depth/object_detection", 1);

    f = boost::bind(&ImageConverter::callback,this, _1, _2);
    server.setCallback(f);
    //Window to open image 
    cv::namedWindow(OPENCV_WINDOW);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }



  void erosion16(){
    /***********************************
     * Erosion16:
     * Erosion for  depth image
     * 16 bits 1 canal
     * Add black pixels when pixels are near
     * *********************************/
    //Temp image to save erosrion changes
    cv::Mat imageTemp (image.rows,image.cols,CV_16UC1);
    erosion16Cuda (image,imageTemp,maskR);
	//cudamain();
	
  }
  void dilation16(){
    /***********************************
     * Dilation16:
     * Dilation for  depth image
     * 16 bits 1 canal
     * Eliminate black pixels when pixels are near
     * *********************************/
    cv::Mat imageTemp (image.rows,image.cols,CV_16UC1);
    //dilation16Cuda (image,imageTemp,maskR);
  }

  void opening16(){
      /***********************************
     * Opening16:
     * Opening for  depth image
     * 16 bits 1 canal
     * Get the opening of the depth images
     * *********************************/
    erosion16();
    dilation16();
  }
  void closing16(){
      /***********************************
     * Closing16:
     * closing for  depth image
     * 16 bits 1 canal
     * Get the closing of the depth images
     * *********************************/
    dilation16();
    erosion16();
  }
  void edge16() {
      this->imageEdge = cv::Mat(image.rows,image.cols,CV_8UC1);
      //edge16Cuda (image,imageEdge,varThreshold16);
	}

  void erosion8(){
    /***********************************
     * Erosion8:
     * Erosion for  edge image
     * 8 bits 1 canal
     * Add black pixels when pixels are near
     * *********************************/
    //Temp image to save erosrion changes
    cv::Mat imageTemp (this->imageEdge.rows,this->imageEdge.cols,CV_8UC1);
    //erosion8Cuda(imageEdge,imageTemp,maskR);
  }



  void dilation8(){
    /***********************************
     * Dilation8:
     * Dilation for  edge image
     * 8 bits 1 canal
     * Eliminate black pixels when white ixels are near
     * *********************************/
    //Temp image to save dilation changes
    cv::Mat imageTemp (this->imageEdge.rows,this->imageEdge.cols,CV_8UC1);
    //dilation8Cuda(imageEdge,imageTemp,maskR);
  }

  void opening8(){
      /***********************************
     * Opening8:
     * Opening for  depth image
     * 8 bits 1 canal
     * Get the opening of the depth images
     * *********************************/
    erosion8();
    dilation8();
  }
  void closing8(){
      /***********************************
     * Closing8:
     * closing for  depth image
     * 8 bits 1 canal
     * Get the closing of the depth images
     * *********************************/
    dilation8();
    erosion8();
  }





  void imageProcesingForDepthImage(){
      /***********************************
     * imageProcesingForDepthImage:
     * imageProcesingForDepthImage
     * 16 bits 1 canal to 8 bit canal
     * Cleans depth image and makes edge detection
     * *********************************/
    //Execute dilation16 n times

    start_timer();
    for(int i = 0;i<vardilation16;i++){
      dilation16();
    }
    msdilation16=msdilation16+stop_timer();
    start_timer();
    //Execute erosion16 n times
    for(int i = 0;i<varerosion16;i++){
      erosion16();
    }
    mserosion16=mserosion16+stop_timer();
    start_timer();
    //Execute opening16 n times
    for(int i = 0;i<varopening16;i++){
      opening16();
    }
    msopening16=msopening16+stop_timer();
    start_timer();
    //Execute closing16 n times
    for(int i = 0;i<varclosing16;i++){
      closing16();
    }
    msclosing16=msclosing16+stop_timer();
    start_timer();
    //Execute edge detection by a Threshold
    edge16();
    msThreshold16=msThreshold16+stop_timer();
    ms16= msdilation16+mserosion16+msopening16+msclosing16+msThreshold16;
  }



void imageProcesingForEdgeImage(){
      /***********************************
     * imageProcesingForEdgeImage:
     * imageProcesingForEdgeImage
     * 8 bits 1 canal
     * Cleans edge image and detecs obstacles
     * *********************************/
    //Execute dilation8 n times
    start_timer();
    for(int i = 0;i<vardilation8;i++){
      dilation8();
    }
    msdilation8=msdilation8+stop_timer();
    start_timer();
    //Execute opening8 n times
    for(int i = 0;i<varopening8;i++){
      opening8();
    }  
    msopening8=msopening8+stop_timer();
    start_timer();
    //Execute closing8 n times
    for(int i = 0;i<varclosing8;i++){
      closing8();
    }
    msclosing8=msclosing8+stop_timer();
    start_timer();
        //Execute erosion8 n times
    for(int i = 0;i<varerosion8;i++){
      erosion8();
    }   
    mserosion8=mserosion8+stop_timer();
    ms8 = msdilation8+msopening8+msclosing8+mserosion8;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    /***********************************
     * imageCb:
     * imageCb is the code that is executed every time a new depth image is recived 
     * *********************************/

    // Try image convertion
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg),"16UC1";
      image     = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //Get the param fron the launcher
    //16 bits

    msCounter++;
    ROS_INFO("Total16= %Lf:Dilation16=%Lf,Erosion16=%Lf,Opening16==%Lf,Closing16=%Lf,Threshold16=%Lf",(ms16/msCounter),(msdilation16/msCounter),(mserosion16/msCounter),(msopening16/msCounter),(msclosing16/msCounter),(msThreshold16/msCounter));
    ROS_INFO("Total8= %Lf:Dilation8=%Lf,Erosion8=%Lf,Opening8==%Lf,Closing8=%Lf",(ms8/msCounter),(msdilation8/msCounter),(mserosion8/msCounter),(msopening8/msCounter),(msclosing8/msCounter));
    ROS_INFO("Total=%Lf",((ms8+ms16)/msCounter));
    //Make image procesing for depth image
    //16 bits
    imageProcesingForDepthImage();
    //o bits
    imageProcesingForEdgeImage();
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, this->image);
    cv::waitKey(3);

    // Publisg modified video stream
    //    sensor_msgs::ImagePtr msgs = cv_bridge::CvImage(cv_ptr->header, "CV_8UC1", this->imageEdge).toImageMsg();
    //image_pub_.publish(msgs);
  }
  void callback(object_detection::object_detectionConfig &config, uint32_t level) {
  /*
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
            */
    vardilation16   =config.dilation16;
    varerosion16    =config.erosion16;
    varopening16    =config.opening16;
    varclosing16    =config.closing16;
    varThreshold16  =config.threshold16;
    //8 bits
    vardilation8  =config.dilation8;
    varerosion8   =config.erosion8;
    varopening8   =config.opening8;
    varclosing8   =config.closing8;
}

};

int main(int argc, char** argv)
{

  // Define ROS node
  ros::init(argc, argv, "image_converter");
  // Create a variable of ImageConverter class
  ImageConverter ic;



  
  //erosion16Cuda();



  // Execute the code in bucle
  ros::spin();
  return 0;
}