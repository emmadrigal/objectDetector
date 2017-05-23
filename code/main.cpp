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


void displayMatches(cv::Mat tracking_img, cv::Mat video, std::vector<cv::KeyPoint> tracking_keypoints, std::vector<cv::KeyPoint> video_keypoints, std::vector< cv::DMatch > matches, int tracking_rows){   
    cv::Size sz1 = tracking_img.size();
    cv::Size sz2 = video.size();
    
    
    cv::Mat img_matches(sz1.height, sz1.width+sz2.width, CV_8UC3);
    
    cv::Mat left(img_matches, cv::Rect(0, 0, sz1.width, sz1.height));
    tracking_img.copyTo(left);
    cv::Mat right(img_matches, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
    video.copyTo(right);
    
    
    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < tracking_rows; i++ ){
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    std::vector< cv::DMatch > good_matches;

    //TODO get min value from config file
    for( int i = 0; i < tracking_rows; i++ ){
        if( matches[i].distance <= std::max(2*min_dist, 0.01 ) )
            good_matches.push_back( matches[i]);
    }
    
    std::cout << "distance: \t";
    std::cout << good_matches[0].distance << std::endl;
    std::cout << "query descriptor index: \t";
    std::cout << good_matches[0].imgIdx << std::endl;
    std::cout << "train descriptor index: \t";
    std::cout << good_matches[0].queryIdx << std::endl;
    std::cout << "train image index: \t";
    std::cout << good_matches[0].trainIdx << std::endl << std::endl;
    
    
    
    cv::imshow( "Good Matches", img_matches );
    
}

void display(){
    cv::Mat cameraMatrix, distCoeffs;
    
    //Reads data from the calibration
    readCalibrationParams(&cameraMatrix, &distCoeffs);
    
    
    cv::VideoCapture cap(0); // open the default camera
    
    if(!cap.isOpened())  // check if we succeeded
        return;

    //Object to extract Keypoints from the reference image
    cv::FeatureDetector* detector;
    
    int minHessian = 800;

    
    //TODO change this according to the config file
    if(true){
        //TODO do this better
        //detector = new cv::FastFeatureDetector();
        //detector = detector->create("FAST");
        detector = new cv::SurfFeatureDetector( minHessian );

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
    else{
        matcher = new cv::BFMatcher();
    }
    
    //Object to store the tracking descriptors
    cv::Mat traking_descriptor, img_descriptor;
    
    //Object to store the frame
    cv::Mat frame, reference_img;
    
    //Objcet to store the interest points
    std::vector<cv::KeyPoint> reference_keypoints;
    
    
    //Flags to indicate what should be shown
    bool tracking = false;
    bool show_matches = true;
    bool show_homography = true;
    bool show_axis = true;
    
    for(;;)
    {    
        cap >> frame; // get a new frame from camera        

        //If space bar is pressed it creates the descriptors for the selected image
        int pressed_key = cv::waitKey(1);
        if(pressed_key == 32){//Spacebar   
            //Takes the keypoints from the image and saves them in the structure
            detector->detect(frame, reference_keypoints);         
            
            //Extracts the keypoints from the structre
            extractor->compute(frame, reference_keypoints, traking_descriptor);
            
            //Clears previous descriptor
            matcher->clear();
            
            //Indicates the program that it can start comparing both images
            tracking = true;
            
            frame.copyTo(reference_img);
            cv::destroyWindow("video");
        }
        else if(pressed_key == 27){//Esc
            std::cout << "Exiting Program" << std::endl;
            return;
        }
        else if(pressed_key == 109){//m
            show_matches = !show_matches;
            if(show_matches){
                std::cout << "Now showing matches" << std::endl;
            }
            else{
                std::cout << "Now hiding matches" << std::endl;
            }
            
        }
        else if(pressed_key == 97){//a
            show_axis = !show_axis;
            if(show_axis){
                std::cout << "Now showing axis" << std::endl;
            }
            else{
                std::cout << "Now hiding axis" << std::endl;
            }
        }
        else if(pressed_key == 104){//h
            show_homography = !show_homography;
            if(show_homography){
                std::cout << "Now showing object visualization" << std::endl;
            }
            else{
                std::cout << "Now hiding object visualization" << std::endl;
            }
        }
        /*
        else{
            if(pressed_key != -1)
                std::cout << pressed_key << std::endl;
        }
        */
        
        //Compares the taken image against the image that is currently displayed
        if(tracking){

            //Stores the interest points
            std::vector<cv::KeyPoint> scene_keypoints;
            
            //Takes the keypoints from the image and saves them in the structure
            detector->detect(frame, scene_keypoints);            
            
            //Extracts the keypoints from the structre
            extractor->compute(frame, scene_keypoints, img_descriptor);
        
            //Stores the matches
            std::vector< cv::DMatch > matches;
            
            //Matches the detected descriptors with the saved descriptors
            matcher->match(traking_descriptor, img_descriptor,  matches);
            
            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< cv::DMatch > good_matches;
            
            double max_dist = 0; double min_dist = 100;
            
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < traking_descriptor.rows; i++ ){
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }


            for( int i = 0; i < traking_descriptor.rows; i++ ){
                if( matches[i].distance < 2*min_dist ){
                    good_matches.push_back( matches[i]);
                }
            }

            cv::Size sz1 = reference_img.size();


            cv::Mat img_matches(sz1.height, sz1.width+sz1.width, CV_8UC3);            
            
            if(show_matches){
                cv::drawMatches(reference_img, reference_keypoints, frame, scene_keypoints, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS  );
                /*
                displayMatches(reference_img, frame, reference_keypoints, scene_keypoints, matches, traking_descriptor.rows);
                */
            }
            else{
                cv::Mat left(img_matches, cv::Rect(0, 0, sz1.width, sz1.height));
                reference_img.copyTo(left);
                cv::Mat right(img_matches, cv::Rect(sz1.width, 0, sz1.width, sz1.height));
                frame.copyTo(right);
            }
            
            
//////////////////////////////////////////////////////////////////////////////////////////////////
            //-- Localize the object
            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;
            
            if(show_homography  || show_axis){
                for( int i = 0; i < good_matches.size(); i++ )
                {
                    //-- Get the keypoints from the good matches
                    obj.push_back( reference_keypoints[ good_matches[i].queryIdx ].pt );
                    scene.push_back( scene_keypoints[ good_matches[i].trainIdx ].pt );
                }


                if( good_matches.size() > 3){
                    //Finds homography
                    cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );
                    
                    
                    if(show_homography){
                        //-- Get the corners from the image_1 ( the object to be "detected" )
                        std::vector<cv::Point2f> obj_corners(4);
                        obj_corners[0] = cvPoint(0,0);
                        obj_corners[1] = cvPoint( reference_img.cols, 0 );
                        obj_corners[2] = cvPoint( reference_img.cols, reference_img.rows );
                        obj_corners[3] = cvPoint( 0, reference_img.rows );
                        
                   
                        std::vector<cv::Point2f> scene_corners(4);
                        //TODO esto se podría cambiar por projectPoints para utilizar la matriz de cámara
                        cv::perspectiveTransform( obj_corners, scene_corners, H);

                        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                        cv::line( img_matches, scene_corners[0] + cv::Point2f( reference_img.cols, 0), scene_corners[1] + cv::Point2f( reference_img.cols, 0), cv::Scalar(0, 255, 0), 4 );
                        cv::line( img_matches, scene_corners[1] + cv::Point2f( reference_img.cols, 0), scene_corners[2] + cv::Point2f( reference_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
                        cv::line( img_matches, scene_corners[2] + cv::Point2f( reference_img.cols, 0), scene_corners[3] + cv::Point2f( reference_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
                        cv::line( img_matches, scene_corners[3] + cv::Point2f( reference_img.cols, 0), scene_corners[0] + cv::Point2f( reference_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
                    }
                    if(show_axis){
                        //TODO implement show_axis representation
                    }
                    
                }
            }

                
                
            //-- Show detected matches
            cv::imshow( "Good Matches & Object detection", img_matches );
//////////////////////////////////////////////////////////////////////////////////////////////////
            
            
        }
        else{
            cv::imshow("video", frame);
        }
    }
}



int main(int ac, char* av[]){
    //TODO read from config file
    
    //TODO do boost program options here
    display();


    return 0;
}
