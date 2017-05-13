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
        //TODO FLANN fails when there are no matches???
        matcher = new cv::FlannBasedMatcher();
    }
    
    //Object to store the tracking descriptors
    cv::Mat traking_descriptor, img_descriptor;
    
    //Object to store the frame
    cv::Mat frame, tracked_img;
    
    //Objcet to store the interest points
    std::vector<cv::KeyPoint> tracked_img_key;
    
    for(;;)
    {    
        cap >> frame; // get a new frame from camera        

        //If space bar is pressed it creates the descriptors for the selected image
        if(cv::waitKey(1) == 32){           
            //Takes the keypoints from the image and saves them in the structure
            detector->detect(frame, tracked_img_key);         
            
            //Extracts the keypoints from the structre
            extractor->compute(frame, tracked_img_key, traking_descriptor);
            
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
            matcher->match(traking_descriptor, img_descriptor,  matches);
            
            
            //TODO this is coppied from the internet, must be checed
            double max_dist = 0; double min_dist = 100;

            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < traking_descriptor.rows; i++ )
            { double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
            }

            //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
            //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
            //-- small)
            //-- PS.- radiusMatch can also be used here.
            std::vector< cv::DMatch > good_matches;

            //TODO get min value from config file
            for( int i = 0; i < traking_descriptor.rows; i++ )
            { if( matches[i].distance <= std::max(2*min_dist, 0.01 ) )
            { good_matches.push_back( matches[i]); }
            }
            
            //-- Draw only "good" matches
            cv::Mat img_matches;
            //TODO get one of this images to be static
            cv::drawMatches(tracked_img, tracked_img_key, frame, reference_keypoints,
                       good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1) );

            //-- Show detected matches
            cv::imshow( "Good Matches", img_matches );            
        }
        else{
            cv::imshow("video", frame);
        }
    }
}



int main(int ac, char* av[]){
    //TODO read from config file
    display();


    return 0;
}
