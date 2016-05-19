#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>  
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> 
#include <opencv2/opencv.hpp>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";
bool found_buttons = false;
vector<Vec3f> buttons;
int whiteCountAvg [] = {0,0,0,0,0,0,0};
int floors [] = {5, 6, 7, 2, 3, 4, 1};
int count = 0;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        //Commented out image_sub is for the kinect color image
        // image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1,
        //Image sub below is used to connect to the image displayed by the usb_cam
        image_sub_ = it_.subscribe("/usb_camera/image_raw", 1,  
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat src, src_gray, btn_finder;
        /// Cloning the read image into a Mat format and assigning it to the variable src
        src = cv_ptr->image.clone();

        if( !src.data )
        { return; }

        /*       //Applies erosion image filter
                 IplImage* img = new IplImage(src);
                 cvErode(img, img, 0, 2);
                 src = Mat(img, true);
         */ 

        //Convert src to a grayscale image and storing it in the src_gray variable
        cvtColor( src, src_gray, CV_BGR2GRAY );

        //Occurs once every frame until we know we have found all 10 buttons
        if(!found_buttons)
        {
            //Threshold the grayscale to only keep the black pixels, which are the black circles next to the actual buttons
            inRange(src_gray, cv::Scalar(10, 10, 10), cv::Scalar(80, 80, 80), btn_finder); 

            /// Reduce the noise so we avoid false circle detection
            GaussianBlur(btn_finder, btn_finder, Size(9, 9), 1, 1);

            /*//
              Applies erosion image filter
              IplImage* img = new IplImage(src_gray);
              cvErode(img, img, 0, 2);
              src_gray = Mat(img, true);
             */

            vector<Vec3f> circles;

            /// Apply the Hough Transform to find the circles and stores them into the circles vector
            HoughCircles( btn_finder, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 50, 35, 0, 0 );

            //For loop checks to make sure that we are only adding circles that have not been seen before to the buttons vector	
            for(int i = 0; i < circles.size(); i++)
            {
                bool exists = false;	
                for(int j = 0; j < buttons.size(); j++)
                {
                    int radius = cvRound(buttons[i][2]);
                    
                    //If statement checks to see if the center of the found circle is within 3 radii of any of the virvles already found. If this is the case
                    //we do not add the circle to the vector
                    if(cvRound(circles[i][0]) > cvRound(buttons[j][0]) - 3* radius && cvRound(circles[i][0]) < cvRound(buttons[j][0] + 3 * radius) &&
                            cvRound(circles[i][1]) > cvRound(buttons[j][1]) - 3* radius && cvRound(circles[i][1]) < cvRound(buttons[j][1] + 3 * radius))
                        exists = true;
                } 
                if(!exists)
                {
                    buttons.push_back(circles[i]);
                }	
            }

            ROS_INFO("%ld", buttons.size());

            //If all buttons have been found
            if(buttons.size() == 10)
            {
                found_buttons = true;
                int max = 0;

                //Loop to find the circles to find the max radius
                for(size_t i = 0; i < buttons.size(); i++)
                {
                    if(buttons.at(i)[2] > max)
                        max = buttons.at(i)[2];		
                }

                //Readjust the raius of each button to the max size at the moment (usually this is the closest to the button size)
                for(size_t i = 0; i < buttons.size(); i++)
                {
                    buttons.at(i)[2] = max; //[2] is the index of the radius of the circle
                    //we origionally only sense the black circles next to the buttons, and move over in the image by adding 2 * radius to
                    //give us the approcximate location of the actual buttons
                    buttons.at(i)[0] += 2 * buttons.at(i)[2];
                }

                //Identify which button is which
                //Sorting algorithm for y cooridinates of the button center
                //Selection Sort : Sorts in Ascending order
                //post = we know the last 7 are buttons with floors attributed to them
                int i, j, first;
                Vec3f temp;
                for(int i= 0; i < buttons.size(); i++)
                {
                    int first = i;
                    for(int j= i; j < buttons.size(); j++)
                    {
                        if(buttons.at(j)[1] < buttons.at(first)[1]) //[1] = y coordinate
                            first = j;
                    }
                    temp = buttons.at(i);   // Swap smallest found with element in position i.
                    buttons.at(i) = buttons.at(first);
                    buttons.at(first) = temp;
                }		

                //Pre: we know that the last index has the greatest Y, meaning it is the lowest on the screen
                //Post: buttons vector only contains buttons that have floors associated with them
                //Removes the buttons at the bottom which are for controling the door and alarm
                int  radi_of_bottom = cvRound(buttons.at(buttons.size() - 1)[2]);
                int  y_of_bottom = cvRound(buttons.at(buttons.size() - 1)[1]);
                buttons.erase(buttons.end());
                for(size_t i = 0; i < buttons.size(); i++)
                {
                    if(buttons.at(i)[1] >  y_of_bottom - 2 * radi_of_bottom && buttons.at(i)[1] <  y_of_bottom + 2 * radi_of_bottom)
                    {
                        buttons.erase(buttons.begin() + i);
                        i--; //TO account for the skip when you erase
                    }
                }

                //Now for each set of three for the last 6, we sort based on x to find each circle's corresponding button
                //Ascending Order
                for(int i= 0; i < 3; i++)
                {
                    int first = i;
                    for(int j= i; j < 3; j++)
                    {
                        if(buttons.at(j)[0] < buttons.at(first)[0]) //[0] = x coordinate
                        {
                            first = j;
                        }
                    }
                    temp = buttons.at(i);   // Swap smallest found with element in position i.
                    buttons.at(i) = buttons.at(first);
                    buttons.at(first) = temp;
                }
                for(int i= 3; i < 6; i++)
                {
                    int first = i;
                    for (int j= i; j < 6; j++)
                    {
                        if(buttons.at(j)[0] < buttons.at(first)[0]) //[0] = x coordinate
                        {
                            first = j;
                        }
                    }
                    temp = buttons.at(i);   // Swap smallest found with element in position i.
                    buttons.at(i) = buttons.at(first);
                    buttons.at(first) = temp;
                }

                //NOW THAT THE BUTTONS HAVE BEEN SORTED
                //WE KNOW
                /*
                 * We now know what floor corresponds to each index of the vector
                 * 
                 * index     =      floor
                 *
                 * 0 = 5
                 * 1 = 6
                 * 2 = 7
                 * 3 = 2
                 * 4 = 3
                 * 5 = 4
                 * 6 = 1
                 */

                //Establises base for white count average int array
                //Threshold to find when the button is lit
                for(int k = 0; k < buttons.size(); k++)
                {
                    int total = 0;
                    int x = cvRound(buttons.at(k)[0]);
                    int y = cvRound(buttons.at(k)[1]);
                    int radius = cvRound(buttons.at(k)[2]);
                    for(unsigned int i = x - radius; i < x + radius; i++){

                        for(unsigned int j = y - radius; j < y + radius; j++)
                        {
                            int b = src_gray.at<Vec3b>(j, i)[0]; // B, g, r will be same because it is a grayscale image	
                            if(b > 220 && b < 255)
                                total++;
                        }
                    }
                    for(int j = 0; j < buttons.size(); j++)
                        whiteCountAvg[k] = total;
                } 
            }
            else if (buttons.size() > 10)
                buttons.clear();
        }
        else 
        {
            count++;
            //Threshold to find when the button is lit
            for(int k = 0; k < buttons.size(); k++)
            {
                int total = 0;
                int x = cvRound(buttons.at(k)[0]);
                int y = cvRound(buttons.at(k)[1]);
                int radius = cvRound(buttons.at(k)[2]);

                for(unsigned int i = x - radius; i < x + radius; i++)
                {
                    for(unsigned int j = y - radius; j < y + radius; j++)
                    {
                        int b = src_gray.at<Vec3b>(j, i)[0]; // B, g, r will be same because it is a grayscale image	
                        if(b > 220 && b < 255)
                            total++;
                    }
                }
                // ROS_INFO("%d, floor %d", total, floors[k]);
                //ROS_INFO("%d", count);
                //After 5 frames
                if(count == 5)
                {
                    if(total > whiteCountAvg[k] - 50 && total < whiteCountAvg[k] - 22)
                        ROS_INFO("Reaching floor %d", floors[k]);
                    whiteCountAvg[k] = total;
                    //ROS_INFO("%d, floor, %d", total, floors[k]);
                }

                //ROS_INFO("Found %d, button %d", total, floors[k]);
                Point center(cvRound(buttons[k][0]), cvRound(buttons[k][1]));
                circle( src_gray, center, radius, Scalar(0,0,255), 5);
                circle( src_gray, center, 3, Scalar(0,0,255), 5);	
            }
            if(count == 5)
                count = 0;
        }
        imshow(OPENCV_WINDOW, src);
        imshow(OUT_WINDOW, src_gray);     
        waitKey(7);
    }

    ///////////////////////////////////////////
};

/** @function main */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "button_finder");
    ImageConverter ic;
    ros::spin();

    return 0;
}

