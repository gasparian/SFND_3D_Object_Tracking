
#include <iostream>
#include <algorithm>
#include <set>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, std::string imgName, bool bWait)
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
    // string windowName = "3D Objects";
    // cv::namedWindow(windowName, 1);
    // cv::imshow(windowName, topviewImg);
    cv::imwrite( "../output/lidar_view_"+imgName, topviewImg );

    if ( bWait )
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> newKptMatches;
    double distMean = 0;
    cv::KeyPoint currKpt, prevKpt;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it) {
        currKpt = kptsCurr.at(it->trainIdx);
        prevKpt = kptsPrev.at(it->queryIdx);
        if ( (currKpt.pt.x >= boundingBox.roi.x) && (currKpt.pt.x <= (boundingBox.roi.x+boundingBox.roi.width)) &&
             (currKpt.pt.y >= boundingBox.roi.y) && (currKpt.pt.y <= (boundingBox.roi.y+boundingBox.roi.height)) ) {
                distMean += it->distance;
                boundingBox.kptMatches.push_back(*it);
        }
    }
    distMean /= boundingBox.kptMatches.size();

    // DEBUG
    cout << "Non-filtered bbox kpt matches: " << boundingBox.kptMatches.size() << endl;
    // DEBUG

    double distTrsh = 0.4;
    for (auto it = boundingBox.kptMatches.begin(); it != boundingBox.kptMatches.end(); ) {
        if ( (std::abs(it->distance - distMean) / distMean) > distTrsh )
            boundingBox.kptMatches.erase(it);
        else
            ++it;
    }

    // DEBUG 
    cout << "Bbox keypoints: " << boundingBox.keypoints.size() 
         << "; Bbox kpt matches: " << boundingBox.kptMatches.size()
         << "; Total kpt matches: " << kptMatches.size() << endl;
    // DEBUG
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    size_t s = distRatios.size(); 
    // only continue if list of distance ratios is not empty
    if (s == 0)
    {
        TTC = NAN;
        return;
    }

    
    double dT = 1. / frameRate;

    // compute camera-based TTC from median distance ratio
    std::sort(distRatios.begin(), distRatios.end());
    double medianDistRatio;
    long medIndex = floor(distRatios.size() / 2.0);
    if (distRatios.size() % 2 == 0)
      medianDistRatio = (distRatios[medIndex] + distRatios[medIndex - 1]) / 2.0;
    else
      medianDistRatio = distRatios[medIndex];

    // DEBUG
    cout << "Median distance ratio: " << medianDistRatio << endl;
    // DEBUG

    // ratio must be > 1 to get the valid, positive TTC 
    // (tracked object must appear bigger on the next frame, if not --> precending vehicle moving faster than ego)
    TTC = -dT / (1 - medianDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1. / frameRate; // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if ( abs(it->y) <= laneWidth / 2.0)
      	  minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
      	if ( abs(it->y) <= laneWidth / 2.0)
          minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }

    // compute TTC from both measurements
    // if minXPrev > minXCurr --> precending vehicle is, probably, slowing down
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

struct bbMatchesCounter {
    int prevBoxId;
    int currBoxId;
    int count;
};

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // DEBUG
    // cout << "Prev frame bboxes: " << prevFrame.boundingBoxes.size() << "; Curr frame bboxes: " << currFrame.boundingBoxes.size() << endl;
    // DEBUG

    double t = (double)cv::getTickCount();
    // search for matched keypoints inside each detected bboxes
    // time complexety is garbage: O(N_matches x N_bboxes^2)
    std::map<int, std::map<int, int>> bbMatchesCount;
    // first fill the `bbMatchesCount` with data: prevFrameIdx : currFrameIdx : <COUNT>
    double intersection_ratio, area_ratio;
    cv::Rect intersection;
    for ( auto match_it=matches.begin(); match_it != matches.end(); ++match_it ) {
        cv::KeyPoint currKpt = currFrame.keypoints.at(match_it->trainIdx);
        cv::KeyPoint prevKpt = prevFrame.keypoints.at(match_it->queryIdx);
        for ( auto currBox_it=currFrame.boundingBoxes.begin(); currBox_it!=currFrame.boundingBoxes.end(); ++currBox_it ) {
            cv::Rect currRoi = currBox_it->roi;
            if ( (currKpt.pt.x >= currRoi.x) && (currKpt.pt.x <= (currRoi.x+currRoi.width)) &&
                 (currKpt.pt.y >= currRoi.y) && (currKpt.pt.y <= (currRoi.y+currRoi.height)) ) {
                    for ( auto prevBox_it=prevFrame.boundingBoxes.begin(); prevBox_it!=prevFrame.boundingBoxes.end(); ++prevBox_it ) {
                        cv::Rect prevRoi = prevBox_it->roi;
                        if ( (prevKpt.pt.x >= prevRoi.x) && (prevKpt.pt.x <= (prevRoi.x+prevRoi.width)) &&
                             (prevKpt.pt.y >= prevRoi.y) && (prevKpt.pt.y <= (prevRoi.y+prevRoi.height)) ) {

                                // keep only boxes which intersects with curr. frame box >= 50%
                                intersection = currBox_it->roi & prevBox_it->roi;
                                area_ratio = currBox_it->roi.area() / prevBox_it->roi.area();
                                intersection_ratio = (double)intersection.area() / currBox_it->roi.area();
                                if ( (intersection_ratio >= 0.5) ) {
                                    bbMatchesCount[prevBox_it->boxID][currBox_it->boxID]++;
                                }
                        }
                    }
            }
        }
    }

    // then compute the maximum count of matched keypoints
    // fill structure with porev, curr. ids and counts
    std::vector<bbMatchesCounter> counts(bbMatchesCount.size());
    for ( auto const &el1 : bbMatchesCount ) {
        for ( auto const &el2 : el1.second ) {
            counts.push_back({el1.first, el2.first, el2.second});
        }
    }

    // iterate through each bbox in current frame 
    // pick only one box from current frame which 
    // corresponds to the box in prev. frame and 
    // drop the other occurences of this current frame box Id
    for ( auto currBox_it=currFrame.boundingBoxes.begin(); currBox_it!=currFrame.boundingBoxes.end(); ++currBox_it ) {
        
        int currBoxId = currBox_it->boxID;
        std::vector<bbMatchesCounter> tmp;
        std::for_each(counts.begin(), counts.end(), 
                      [&tmp, &currBoxId](const bbMatchesCounter& el) {
                          if ( el.currBoxId == currBoxId )
                            tmp.push_back(el);
                      });

        if ( tmp.empty() )
            continue;

        auto maxVal = std::max_element(tmp.begin(), tmp.end(), 
            [](const bbMatchesCounter& p1, const bbMatchesCounter& p2) {
                return p1.count < p2.count;
            }
        );

        bbBestMatches[maxVal->prevBoxId] = maxVal->currBoxId;

        // DEBUG
        // cout << maxVal->prevBoxId << " : " << maxVal->count << " : " << maxVal->currBoxId << endl;
        // DEBUG

        counts.erase(
            std::remove_if(
                counts.begin(), counts.end(), 
                [&maxVal](const bbMatchesCounter& el) { return el.prevBoxId == maxVal->prevBoxId; } 
            ),
            counts.end()
        );
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Bboxes matching done in " << 1000 * t / 1.0 << " ms" << endl;
}
