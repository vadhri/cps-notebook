#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
	long t = 0;
  
    if (matcherType.compare("MAT_BF") == 0)
    {   
        if (descriptorType.compare("DES_HOG")==0)
        {
            matcher = cv::BFMatcher::create(cv::NORM_L2 , crossCheck);
        }
        else
        {
            int normType = cv::NORM_HAMMING;
            matcher = cv::BFMatcher::create(normType, crossCheck);
        }
    }
    else if (matcherType.compare("MAT_FLANN") == 0) {
        if (descSource.type() != CV_32F) {  // opencv bug workaround
            descSource.convertTo(descSource, CV_32F);
        }      

        if (descRef.type() != CV_32F) {  // opencv bug workaround
            descRef.convertTo(descRef, CV_32F);
        }      
                
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    if (selectorType.compare("SEL_NN") == 0) { 
    	matcher->match(descSource, descRef, matches); 
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
         vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        //cout << "no of kp removed = " << knn_matches.size() - matches.size() << endl;
    }
    else {
        //cout << "Suprise !!"<<endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0) {

        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0) {
      extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0) {
      extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0) {
      extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0) {
      extractor = cv::xfeatures2d::SIFT::create();
    }
    else {
      std::////cout << "ERROR in descriptor-type within descKeypoints() ....exitting" << std::endl;
      exit(EXIT_FAILURE);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  	////cout <<"Time taken (Identify) = " << 1000 * t / 1.0 << " ms " << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  	////cout <<"Time taken (Detect)= " << 1000 * t / 1.0 << " ms " << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// This method is prepared from the reference documentation on opencv. 
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    int blockSize = 2;      // for every pixel, a blockSize x blockSize neighbourhood is considered
    int minResponse = 100;  // 8bit scaled response matrix - minimum value for a corner  
    int apertureSize = 3;   // Sobel operator - aperture parameter 
    double k = 0.04;        // Harris parameter 
    double maxOverlap = 0.0; 

    double t = (double)cv::getTickCount();

    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j,i);
            if (response > minResponse) { 
                bool bOverlap=  false;
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.response = response;
              	newKeyPoint.size = 2 * apertureSize;
              
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap  = true;
                        if (newKeyPoint.response > (*it).response)
                        {
                            *it = newKeyPoint;
                            break;
                        }
                    }
                }
                if (!bOverlap) {
                    keypoints.push_back(newKeyPoint);
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    ////cout << "Harris corners  (n = " << keypoints.size() << ")" <<endl;
  	////cout <<"Time taken (Detect) = " << 1000 * t / 1.0 << " ms " << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;
	//cout<<"detectorType = "<<detectorType<<endl;
    if(detectorType == "FAST")
    {
        detector = cv::FastFeatureDetector::create();
    }
    else if (detectorType == "ORB")
    {
        detector = cv::ORB::create();
    }  
    else if (detectorType == "BRISK")
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType == "SIFT")
    {
        detector = cv::xfeatures2d::SIFT::create();
    }  
    else if (detectorType == "AKAZE")
    {
        detector = cv::AKAZE::create();
    }
    else
    {
        ////cout << "Unknown detector" << endl;
    }

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    ////cout << detectorType << " detector (n = " << keypoints.size() << ")" <<endl;
  	////cout <<"Time taken (Detect)=" << 1000 * t / 1.0 << " ms " << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}