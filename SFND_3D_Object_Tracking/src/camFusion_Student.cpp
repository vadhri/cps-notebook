
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;

      	pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; 
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }
        } 
		
       if (enclosingBoxes.size() == 1) { 
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }
    } 
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}  

void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<double> euclideanDistance;
    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
    {
        int currKptIndex = (*it).trainIdx;
        const auto &currKeyPoint = kptsCurr[currKptIndex];

        if(boundingBox.roi.contains(currKeyPoint.pt))
        {
            int prevKptIndex = (*it).queryIdx;
            const auto &prevKeyPoint = kptsPrev[prevKptIndex];

            euclideanDistance.push_back(cv::norm(currKeyPoint.pt - prevKeyPoint.pt));
        }
    }
  
    int pair_num =  euclideanDistance.size();
    double euclideanDistanceMean = std::accumulate(euclideanDistance.begin(), euclideanDistance.end(), 0.0) / pair_num;

    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
    {
        int currKptIndex = (*it).trainIdx;
        const auto &currKeyPoint = kptsCurr[currKptIndex];

        if(boundingBox.roi.contains(currKeyPoint.pt))
        {
            int prevKptIndex = (*it).queryIdx;
            const auto &prevKeyPoint = kptsPrev[prevKptIndex];

            double temp = cv::norm(currKeyPoint.pt - prevKeyPoint.pt);

            double euclideanDistanceMean_Augment = euclideanDistanceMean * 1.3;
            if(temp < euclideanDistanceMean_Augment)
            {
                boundingBox.keypoints.push_back(currKeyPoint);
                boundingBox.kptMatches.push_back(*it);
            }
        }
    }
}


void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    std::vector<double> distanceRatios; 
    double minDistanceThreshold = 100.0; 
    TTC = NAN;
    // numeric limits of double = 2.22045e-16 

    for (auto outerIter = kptMatches.begin(); outerIter != kptMatches.end() - 1; ++outerIter) { 
        cv::KeyPoint kpOuterCurr = kptsCurr.at(outerIter->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(outerIter->queryIdx);

        for (auto innerIter = kptMatches.begin() + 1; innerIter != kptMatches.end(); ++innerIter) { 
            cv::KeyPoint kpInnerCurr = kptsCurr.at(innerIter->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(innerIter->queryIdx);

            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDistanceThreshold){
                distanceRatios.push_back(distCurr / distPrev);
            }   
        } 
    }    

    if (distanceRatios.size() != 0.0) {
        std::sort(distanceRatios.begin(), distanceRatios.end());
        double dT = 1 / frameRate;
        double median = distanceRatios[distanceRatios.size() / 2];
        TTC = -dT / (1 - median);
    }
}

void sortLidarPointsAsc(std::vector<LidarPoint> &lidarPoints)
{
    std::sort(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint a, LidarPoint b) {
        return a.x < b.x;  
    });
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPrevPoints,
                     std::vector<LidarPoint> &lidarCurrPoints, double frameRate, double &TTC)
{
    sortLidarPointsAsc(lidarPrevPoints);
    sortLidarPointsAsc(lidarCurrPoints);
    double dt = 1.0 / frameRate;
    double d1 = lidarCurrPoints[lidarCurrPoints.size() / 2].x; 
    double d0 = lidarPrevPoints[lidarPrevPoints.size() / 2].x;
    TTC = d1 * dt / (d0 - d1);
}
// Calculate IQR
// - Order the data from least to greatest.
// - Find the median.
// - Calculate the median of both the lower and upper half of the data.
// - The IQR is the difference between the upper and lower medians.

double calcIQR(std::vector<double> &distances)
{
    std::sort(distances.begin(), distances.end());
    size_t lowerHalf = distances.size() / 4;
    size_t upperHalf = (3 * distances.size()) / 4;
    return distances[upperHalf] + 1.2*(distances[upperHalf] - distances[lowerHalf]);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bestMatches, DataFrame &prevFrame, DataFrame &currFrame) {
    // Local match count and matches storage to be optimized later.
    map<pair<int, int>, int> tmpMatchCnt;
    map<std::pair<int, int>, std::vector<cv::DMatch>> tmpMatches;

    // Go over all matches and ddo the following. 
    // Check if current keypoint  is there in curent frame boudig boxes. 
    // Check if prev keypoint is there in previous frame bounding boxes.
    // check if prev as current exist - match 

    for (const auto &match : matches) {
        cv::KeyPoint prevKpt = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint currKpt = currFrame.keypoints[match.trainIdx];

        int prevBoxId = -1;
        int currBoxId = -1;

        for (const auto &currBox : currFrame.boundingBoxes) {
            if (currBox.roi.contains(currKpt.pt)) {
                currBoxId = currBox.boxID;
                break;
            }
        }

        for (const auto &prevBox : prevFrame.boundingBoxes) {
            if (prevBox.roi.contains(prevKpt.pt)) {
                prevBoxId = prevBox.boxID;
                break;
            }
        }

        if (prevBoxId != -1 && currBoxId != -1) {
            tmpMatches[make_pair(prevBoxId, currBoxId)].push_back(match);
        }
    }

    //Filter current matches based on Euclidean distance
    for (auto &tmpMatch : tmpMatches) {
        std::vector<double> eu_dist;
        for (const auto &match : tmpMatch.second){
            cv::KeyPoint prevKpt = prevFrame.keypoints[match.queryIdx];
            cv::KeyPoint currKpt = currFrame.keypoints[match.trainIdx];
            eu_dist.push_back(cv::norm(prevKpt.pt - currKpt.pt));
        }
        double threshold = calcIQR(eu_dist);
        std::vector<cv::DMatch> filteredMatches;
        for (size_t i = 0; i < eu_dist.size(); i++){
            if (eu_dist[i] <= threshold){
                filteredMatches.push_back(tmpMatch.second[i]);
                }
            }
            tmpMatch.second = filteredMatches;
            tmpMatchCnt[tmpMatch.first] = tmpMatch.second.size();
        }

    // Find the best matches for each bounding box in the previous frame
    for (const auto &prevBox : prevFrame.boundingBoxes) {
        int bestMatchId = -1;
        int maxCnt = 0;

        for (const auto &count : tmpMatchCnt) {
            if (count.first.first == prevBox.boxID && count.second > maxCnt) {
                bestMatchId = count.first.second;
                maxCnt = count.second;
            }
        }

        if (bestMatchId != -1) {
            bestMatches[prevBox.boxID] = bestMatchId;
        }
    }
}