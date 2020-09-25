/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    ofstream details;
    details.open("../output/helpfulResults.txt"); //stores the output in a filet observe and seleclt the best results possible
    details << "|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |" << endl;
    details << "|:---:|:---:|:----:|:-----:|:-----:|" << endl;
    
    ofstream results;
    results.open("../output/results.csv", ios::out | ios::trunc);
    results << "detector" << "," << "descriptor" << "," << "All Keypoints" << "," << "All matches" << "," << "time s" << endl;

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    bool bVis = false;            // visualize results
    int sr = 0;
    /* MAIN LOOP OVER ALL IMAGES */
    vector<string> detectorS = {"SHITOMASI", "HARRIS", "BRISK", "FAST", "SIFT", "ORB", "AKAZE"};
    vector<string> descriptorS = {"BRISK", "BRIEF", "ORB", "FREAK", "SIFT", "AKAZE"};
    for (auto it1 = detectorS.begin(); it1 != detectorS.end(); ++it1) // for all detectors
        for (auto it2 = descriptorS.begin(); it2 != descriptorS.end(); ++it2)// for all descriptors
        {
            string detType = *it1;
            string descType = *it2;
            int allkeypoints = 0, allmatches = 0;

            vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
            double t = (double)cv::getTickCount();
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) // for all images
            {
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;
                if (dataBuffer.size() >= dataBufferSize)
                    dataBuffer.erase(dataBuffer.begin());
                dataBuffer.push_back(frame);

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                if (detType.compare("SHITOMASI") == 0)
                    detKeypointsShiTomasi(keypoints, imgGray, false);
                else if (detType.compare("HARRIS") == 0)
                    detKeypointsHarris(keypoints, imgGray, bVis);
                else
                    detKeypointsModern(keypoints, imgGray, detType, bVis);

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle)
                {
                    vector<cv::KeyPoint> vehicleKeypoints;
                    for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                    {
                        float x = it->pt.x, y = it->pt.y;
                        if ((x < vehicleRect.x + vehicleRect.width && x >= vehicleRect.x) && (y < vehicleRect.x + vehicleRect.height && y >= vehicleRect.y))
                            vehicleKeypoints.push_back(*it);
                    }
                    keypoints = vehicleKeypoints;
                }
                allkeypoints += keypoints.size();

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 150;
                    if (detType.compare("SHITOMASI") == 0)
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT, SIFT

                if (descType == "AKAZE" && detType != "AKAZE")
                    continue;
                
                else if (detType == "SIFT" && descType == "ORB")
                    continue;
                
                cv::Mat descriptors;
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descType);
                

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    string descriptorType = descType == "SIFT"? "DES_HOG" : "DES_BINARY"; // DES_BINARY, DES_HOG
                    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                        (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                        matches, descriptorType, matcherType, selectorType);
                    allmatches += matches.size();

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    // visualize matches between current and previous image
                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                            (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                            matches, matchImg,
                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                            vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 10);
                        cv::imshow(windowName, matchImg);
                        cout << "Press key to continue to next image" << endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }
            } // eof loop over all images
            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
           
            //log and display the results of this combination (detector + desctriptor)
            
            if (allmatches == 0)
            {
                results << detType << "," << descType << "," << allkeypoints << "," << "" << "," << t << endl;
                continue;
            }
            results << detType << "," << descType << "," << allkeypoints << "," << allmatches << "," << t << endl;


            details << "| " << ++sr << " | "<< detType << " + " << descType << " |" << allkeypoints << " |" << allmatches << " |" << t << " |"<< endl;
            
            cout << "combination: " << detType << " + " << descType << "\n";
            cout << "all keypoints: " << allkeypoints << endl;
            cout << "all matches: " << allmatches << endl;
            cout << "Time: " << t << " s\n------------" << endl;

        }
    
    results.close();
    details.close();
    return 0;
}
