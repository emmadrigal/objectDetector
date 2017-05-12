#include <iostream>

//Used for image manipulation
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/features2d.hpp"



/**
 *Reads the calibration paramaters from the CalibrationParamters file
 *@param cameraMatrix pointer to the camara Matrix to recieve the new data
 *@param distCoeffs pointer to the matrix that will hold the distortion Coefficients
*/
void readCalibrationParams(cv::Mat *cameraMatrix, cv::Mat *distCoeffs)
{
        cv::FileStorage fs( "../CalibrationParameters", cv::FileStorage::READ );
        fs["Camera_Matrix" ] >> *cameraMatrix;
        fs["Distortion_Coefficients"] >> *distCoeffs;
        fs.release();
}

void display(){
    cv::Mat cameraMatrix, distCoeffs;
    
    //Reads data from the calibration
    readCalibrationParams(&cameraMatrix, &distCoeffs);
    
    
    cv::VideoCapture cap(0); // open the default camera
    
    if(!cap.isOpened())  // check if we succeeded
        return;
    
    
    bool tracking = false;
    
    //Object to extract Keypoints from the reference image
    cv::FeatureDetector* detector;
    
    //TODO change this according to the config file
    if(true){
        //TODO do this better
        detector = new cv::FastFeatureDetector();
        //detector = detector->create("FAST");
    }
    
    //Object to extract descriptors from the interest points
    cv::DescriptorExtractor* extractor;

    //TODO change this according to the config file
    if(true){
        //TODO do this better
        extractor = new cv::SurfDescriptorExtractor();
        //extractor = extractor->create("FAST");
    }
    
    
    //Object to match features from reference to the image being analysed
    cv::DescriptorMatcher* matcher;
    //TODO cahnge this according to the config file
    if(true){
        matcher = new cv::FlannBasedMatcher();
    }
    
    //Object to store the tracking descriptors
    cv::Mat traking_descriptor, img_descriptor;
    
    //Object to store the frame
    cv::Mat frame;
    
    //TODO this is tmp
    std::vector<cv::KeyPoint> tracked_img_key;
    
    
    
    for(;;)
    {    
        cap >> frame; // get a new frame from camera        



        
        //If space bar is pressed it creates the descriptors for the selected image
        if(cv::waitKey(1) == 32){
            //Objcet to store the interest points
            std::vector<cv::KeyPoint> reference_keypoints;
            
            //Takes the keypoints from the image and saves them in the structure
            detector->detect(frame, reference_keypoints);         
            
            
            //Extracts the keypoints from the structre
            extractor->compute(frame, reference_keypoints, traking_descriptor);
            
            //Clears previous descriptor
            matcher->clear();
            
            //Indicates the program that it can start comparing both images
            tracking = true;
            
            //TODO this is tmp
            tracked_img = frame;
            
        }
        
        //Compares the taken image against the image that is currently displayed
        if(tracking){

            //Stores the interest points
            std::vector<cv::KeyPoint> reference_keypoints;
            
            //Takes the keypoints from the image and saves them in the structure
            detector->detect(frame, reference_keypoints);            
            
            //Extracts the keypoints from the structre
            extractor->compute(frame, reference_keypoints, img_descriptor);
        
            //Stores the matches
            std::vector< cv::DMatch > matches;
            
            //Matches the detected descriptors with the saved descriptors
            matcher->match(img_descriptor, traking_descriptor, matches);
        }
        
        cv::imshow("axis", frame);
    }
}



int main(int ac, char* av[]){
    //TODO read from config file
    display();


    return 0;
}
