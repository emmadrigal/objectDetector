#include <iostream>
#include <string>

//USED for time measurement
#include <chrono>

//Used for interaction with the user
#include <boost/program_options.hpp>

//Used for image manipulation
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/features2d.hpp"

namespace po = boost::program_options;



/**
 *Reads the configuration parameters for this program
 *@param cameraMatrix pointer to the camara Matrix to recieve the new data
 *@param distCoeffs pointer to the matrix that will hold the distortion Coefficients
*/
void readConfigParams(std::string *method, std::string *matcher, bool* calibrate){
        cv::FileStorage fs( "../config.cfg", cv::FileStorage::READ );
        
        if(fs.isOpened()){
            fs["method" ] >> *method;
            fs["matcher"] >> *matcher;
            fs["calibrate"] >> *calibrate;
            //TODO other parameters can be added here
            fs.release();
        }
        else{
            cv::FileStorage fs( "../config.cfg", cv::FileStorage::WRITE );
            
            fs << "method" << "ORB";
            *method = "ORB";
            fs << "matcher"    << "FLANN";
            *matcher = "FLANN";
            fs << "calibrate"    << false;
            *calibrate = false;
        }
}


/**
 *Reads the calibration paramaters from the CalibrationParamters file
 *@param cameraMatrix pointer to the camara Matrix to recieve the new data
 *@param distCoeffs pointer to the matrix that will hold the distortion Coefficients
*/
void readCalibrationParams(cv::Mat *cameraMatrix, cv::Mat *distCoeffs){
        cv::FileStorage fs( "../CalibrationParameters", cv::FileStorage::READ );
        fs["Camera_Matrix" ] >> *cameraMatrix;
        fs["Distortion_Coefficients"] >> *distCoeffs;
        fs.release();
}

/**
 * Displays the object detection software and detects the different commands
*/
void display(){
    cv::Mat cameraMatrix, distCoeffs;
        
    //Open the default camera
    cv::VideoCapture cap(0); 
    
    //Check if succesfull
    if(!cap.isOpened())  
        return;

    //Object to extract Keypoints from the reference image
    cv::Feature2D* detectorANDextractor;
    //Object to match features from reference to the image being analysed
    cv::DescriptorMatcher* matcher;
    
    std::string which_method;
    std::string which_matcher;
    bool calibrate;
    
    //Read config parameters
    readConfigParams(&which_method, &which_matcher, &calibrate);
    
    if(calibrate){
        //Reads data from the calibration
        readCalibrationParams(&cameraMatrix, &distCoeffs);
    }

    if(     which_method ==   "SURF") {
        detectorANDextractor = new cv::SURF();
    }
    else if(which_method ==   "SIFT") {
        detectorANDextractor = new cv::SIFT();
    }
    else if(which_method ==    "ORB") {
        detectorANDextractor = new cv::ORB();
    }
    else if(which_method ==  "BRISK") {
        detectorANDextractor = new cv::BRISK();
    }
    else{
        detectorANDextractor = new cv::ORB();
        std::cout << "Detector name not recongnized, a default one has been chosen" << std::endl;
    }
    
    
    if(which_matcher   == "FLANN") {matcher = new cv::FlannBasedMatcher();}
    else if(which_matcher == "BF") {matcher = new cv::BFMatcher();}
    else{
        matcher = new cv::FlannBasedMatcher();
        std::cout << "Matcher name not recongnized, a default one has been chosen" << std::endl;
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
    
    float axis_length = 200.0;
    
    cv::Mat distorted_frame;


    
    for(;;){
        // get a new frame from camera
        //This variable is changed from the configuration file
        if(calibrate){
            cap >> distorted_frame; 
            //Transforms frame to compensate for lens distortion based on the calibration matrixes
            cv::undistort(distorted_frame, frame, cameraMatrix, distCoeffs);
        }
        else{
            cap >> frame;
        }

        //If space bar is pressed it creates the descriptors for the selected image
        int pressed_key = cv::waitKey(1);
        if(pressed_key == 32){//Spacebar
            //Takes the keypoints from the image and saves them in the structure
            detectorANDextractor->detect(frame, reference_keypoints);         
            
            //Extracts the keypoints from the structre
            detectorANDextractor->compute(frame, reference_keypoints, traking_descriptor);
            
            //Clears previous descriptor
            matcher->clear();
            
            //Indicates the program that it can start comparing both images
            tracking = true;
            
            //Only display the new frame
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
        else if((pressed_key == 111) && (tracking) && show_axis){//o
            if(axis_length < 600)
                axis_length = axis_length + 20;
        }
        else if((pressed_key == 108) && (tracking) && show_axis){//l
            if(axis_length > 20)
                axis_length = axis_length - 20;
        }
        
        //Compares the taken image against the image that is currently displayed
        if(tracking){

            //Stores the interest points
            std::vector<cv::KeyPoint> scene_keypoints;
            
            //Takes the keypoints from the image and saves them in the structure
            detectorANDextractor->detect(frame, scene_keypoints); 
            
            //Extracts the keypoints from the structre
            detectorANDextractor->compute(frame, scene_keypoints, img_descriptor);
            
            //This is used to provide support for ORB descritors with FLANN matcher
            if(img_descriptor.type()!=CV_32F) {
                img_descriptor.convertTo(img_descriptor, CV_32F);
                traking_descriptor.convertTo(traking_descriptor, CV_32F);
            }

        
            //Stores the matches
            std::vector< cv::DMatch > matches;           
            
            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< cv::DMatch > good_matches;
            
            //Matches the detected descriptors with the saved descriptors
            matcher->match(traking_descriptor, img_descriptor,  matches);
            
            double max_dist = 0; double min_dist = 10000;
            
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < traking_descriptor.rows; i++ ){
                double dist = matches[i].distance;
                if(dist < min_dist) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            //Only get the descriptors that are no more than 3 times bigger than the smaller one, but not bigger than half the largerst
            //This is to compensate for the different algorithms
            for( int i = 0; i < traking_descriptor.rows; i++ ){
                if( matches[i].distance < std::min(3*min_dist, max_dist/2) ){
                    good_matches.push_back( matches[i]);
                }
            }
            
            //Get size to build new image
            cv::Size sz1 = reference_img.size();

            //Build new image, reference image next to the video
            cv::Mat img_matches(sz1.height, sz1.width+sz1.width, CV_8UC3);            
            cv::Mat left(img_matches, cv::Rect(0, 0, sz1.width, sz1.height));
            reference_img.copyTo(left);
            cv::Mat right(img_matches, cv::Rect(sz1.width, 0, sz1.width, sz1.height));
            frame.copyTo(right);
            
            //Only display matches is there are more than 5
            if(good_matches.size() > 5){
            
                //Display lines between the matches in both images
                if(show_matches){
                    cv::drawMatches(reference_img, reference_keypoints, frame, scene_keypoints, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS  );
                }
                
                
                //-- Localize the object
                std::vector<cv::Point2f> obj;
                std::vector<cv::Point2f> scene;
                
                //This comparition is made in order to calculate homography
                if(show_homography  || show_axis){
                
                    //Get reference points for the object an the new perspective of the object
                    for( int i = 0; i < good_matches.size(); i++ ){
                        //-- Get the keypoints from the good matches
                        obj.push_back( reference_keypoints[ good_matches[i].queryIdx ].pt );
                        scene.push_back( scene_keypoints[ good_matches[i].trainIdx ].pt );
                    }


                    //Finds homography
                    //TODO get method from config file
                    cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );
                    
                    //-- Get the corners from the image_1 ( the object to be "detected" )
                    std::vector<cv::Point2f> obj_corners(4);
                    obj_corners[0] = cvPoint(0,0);
                    obj_corners[3] = cvPoint( reference_img.cols, 0 );
                    obj_corners[2] = cvPoint( reference_img.cols, reference_img.rows );
                    obj_corners[1] = cvPoint( 0, reference_img.rows );
               
                    std::vector<cv::Point2f> scene_corners(4);
                    
                    cv::perspectiveTransform( obj_corners, scene_corners, H);
                    
                    
                    if(show_homography){
                        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                        cv::line( img_matches, scene_corners[0] + cv::Point2f( reference_img.cols, 0), scene_corners[1] + cv::Point2f( reference_img.cols, 0), cv::Scalar(255, 255, 255), 4 );
                        cv::line( img_matches, scene_corners[1] + cv::Point2f( reference_img.cols, 0), scene_corners[2] + cv::Point2f( reference_img.cols, 0), cv::Scalar(255, 255, 255), 4 );
                        cv::line( img_matches, scene_corners[2] + cv::Point2f( reference_img.cols, 0), scene_corners[3] + cv::Point2f( reference_img.cols, 0), cv::Scalar(255, 255, 255), 4 );
                        cv::line( img_matches, scene_corners[3] + cv::Point2f( reference_img.cols, 0), scene_corners[0] + cv::Point2f( reference_img.cols, 0), cv::Scalar(255, 255, 255), 4 );
                    }
                    
                    if(show_axis){
                        cv::Mat rvec, tvec;
                        
                        //Converts the obj reference to 3d assuming that it is 
                        std::vector<cv::Point3f> obj_3D;
                        
                        for(int i = 0; i < obj_corners.size(); i++){
                            obj_3D.push_back(cv::Point3f(obj_corners[i].x, obj_corners[i].y, 0));  
                        }
                        
                        //Solve PnP based on the homography
                        bool solved = cv::solvePnP(obj_3D, scene_corners, cameraMatrix, distCoeffs, rvec, tvec);
                        if (solved){
                            std::vector<cv::Point3f> axis;//Vector de Vectores que describe como se debería ver el espacio
                            
                            axis.push_back(cv::Point3f(0, 0, 0));               //Origin
                            axis.push_back(cv::Point3f(axis_length, 0, 0));     //X axis end
                            axis.push_back(cv::Point3f(0, axis_length, 0));     //Y axis end
                            axis.push_back(cv::Point3f(0, 0, -axis_length));     //Z axis end
                            
                            std::vector<cv::Point2f> imagePoints;  //Recibe el vector transformado
                            cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
                            
                            cv::line(img_matches, imagePoints[0]+ cv::Point2f( reference_img.cols, 0), imagePoints[1]+ cv::Point2f( reference_img.cols, 0), cv::Scalar(0, 255, 0), 4 , 8);
                            cv::line(img_matches, imagePoints[0]+ cv::Point2f( reference_img.cols, 0), imagePoints[2]+ cv::Point2f( reference_img.cols, 0), cv::Scalar(255, 0, 0), 4 , 8);
                            cv::line(img_matches, imagePoints[0]+ cv::Point2f( reference_img.cols, 0), imagePoints[3]+ cv::Point2f( reference_img.cols, 0), cv::Scalar(0, 0, 255), 4 , 8);
                        }            
                    }
                }
            }
                
            //-- Show detected matches
            cv::imshow( "Good Matches & Object detection", img_matches );
        }
        else{
            cv::imshow("video", frame);
        }
    }
}



int main(int ac, char* av[]){    
    try{
        po::options_description desc("Allowed Options");
        desc.add_options()
                ("help", "Produce help message")
                ("display", "Allows for feature detection")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if(vm.count("help")){
            std::cout << "Spacebar -> Capture reference image\nEsc      -> Exit program\nm        -> Toggle matches\nh        -> Toggle object detection\na        -> Toggle Axis\no\t -> Increase Axis Size\nl\t -> Decrease Axis Size\n";
            return 0;
        }
        else if(vm.count("display")){
            display();       
        }
        else {
            std::cout << desc << "\n";
            return 0;
        }

    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
