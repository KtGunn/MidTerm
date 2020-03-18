/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

///****************************************************************************************
/// VERIFY input
//

typedef enum {mat, desc, dt, sel} enmType;

bool legitSelection (string cand, enmType type) {

  static vector<string> vDescriptors = {"BRIEF", "ORB", "FREAK", "AKAZE", "SIFT", "BRISK"};
  static vector<string> vDetectors = {"SHITOMASI", "HARRIS", "BRISK", "FAST", "ORB", "AKAZE", "SIFT"};
  static vector<string> vMatchers = {"MAT_BF", "MAT_FLANN"};
  static vector<string> vSelectors = {"SEL_NN", "SEL_KNN"};

  vector<string> vLabels;
  switch (type) {
  case mat:
    { vLabels = vMatchers; }
    break;
  case sel:
    { vLabels = vSelectors; }
    break;
  case desc:
    { vLabels = vDescriptors; }
    break;
  case dt:
    { vLabels = vDetectors; }
    break;
  default:
    {cout << "Bad selection type. Bye...\n";
      exit (-1);
    }
  }
  for (auto& legit : vLabels) {
    if (legit.compare (cand) == 0) {
	    return (true);
    }
  }
  return (false);
}



///****************************************************************************************
/// MAIN PROGRAM
//
int main(int argc, const char *argv[])
{

  ///////////////////////////////////////////////////////
  // Mid term task results are logged to file
  ofstream outFID;
  outFID.open ("MidTerm.log", ios::out | ios::app);

  

  const char *keys =
    "{help h || help messge}" 
    "{detector det dt|| keypoint detector type}" 
    "{descriptor desc ds || keypoint descriptor type}" 
    "{matcher mat ma || keypoint matcher type [MAT_BF, MAT_FLANN]}" 
    "{selector sel || selector type [SEL_NN, SEL_KNN]}"
    ;
  
  enmType inputType;
  cv::CommandLineParser parser (argc, argv , keys);
  
  /////////////////////////////////////////////////////////////
  /// DETECTOR
  string detectorType  = parser.get<string>("detector");
  
  if (detectorType.empty() || !legitSelection (detectorType, dt)) {
    cout << " Detector type '" << detectorType << "' is not acceptable\n";
    return (0);
  }
  cout << " DETECTOR TYPE '" << detectorType << "'\n";
  
  
  /////////////////////////////////////////////////////////////
  /// DESCRIPTOR
  string descriptorType = parser.get<string>("descriptor"); // BRIEF, ORB, FREAK, AKAZE, SIFT
  
  if (descriptorType.empty() || !legitSelection (descriptorType, desc)) {
    cout << " Descriptor type '" << descriptorType << "' is not acceptable\n";
    return (0);
  }
  cout << " DESCRIPTOR TYPE '" << descriptorType << "'\n";


  /////////////////////////////////////////////////////////////
  /// MATCHER
  string matcherType = parser.get<string>("matcher"); // BRIEF, ORB, FREAK, AKAZE, SIFT
  
  if (matcherType.empty() || !legitSelection (matcherType, mat)) {
    cout << " Matcher type '" << matcherType << "' is not acceptable\n";
    return (0);
  }
  cout << " Matcher TYPE '" << matcherType << "'\n";
  


  /////////////////////////////////////////////////////////////
  /// SELECTOR
  string selectorType = parser.get<string>("selector"); // BRIEF, ORB, FREAK, AKAZE, SIFT
  
  if (selectorType.empty() || !legitSelection (selectorType, sel)) {
    cout << " Selector type '" << selectorType << "' is not acceptable\n";
    return (0);
  }
  cout << " Selector TYPE '" << selectorType << "'\n";

  
  ////////////////////////////////////////////////////////////
  /// RESULTS FILE HEADER
  //
  stringstream header;
  header << endl << "DETECTOR " << detectorType << " DESCRIPTOR " << descriptorType
         << " MATCHER " << matcherType << " SELECTOR " << selectorType << "'\n";

  outFID << header.str ();



  /* INIT VARIABLES AND DATA STRUCTURES */
  
  // data location
  string dataPath = "../";
  
  // camera
  string imgBasePath = dataPath + "images/";
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
  string imgFileType = ".png";
  
  int imgStartIndex = 0; // first file index to load
  // (assumes Lidar and camera names have identical naming convention)
  int imgEndIndex = 9;   // last file index to load
  int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)
  
  // misc
  int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
  vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
  bool bVis = false;            // visualize results
  
  /* MAIN LOOP OVER ALL IMAGES */
  
  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
      ///********************************************************************************
      /// LOAD IMAGE INTO BUFFER
      //
      // assemble filenames for current index
      ostringstream imgNumber;
      imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
      string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
      
      // load image from file and convert to grayscale
      cv::Mat img, imgGray;
      img = cv::imread(imgFullFilename);
      cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

      //// STUDENT ASSIGNMENT
      //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
      
      // Push image into data frame buffer
      DataFrame frame;
      frame.cameraImg = imgGray;
      
      // [KTG] Buffer Implementation
      if ( dataBuffer.size() >= dataBufferSize )
        {
          vector<DataFrame>::iterator it = dataBuffer.begin()+1;
          for (short n=1; n<dataBufferSize;n++)
            {
              *(it-1) = *it; // Shuffle backward
              it++;
            }
          *(it-1) = frame; // Assign at back/last position
        } else {
        dataBuffer.push_back(frame); // Insert normally
      }
      
      //// EOF STUDENT ASSIGNMENT
      cout << endl << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
      
      /* DETECT IMAGE KEYPOINTS */
      
      // extract 2D keypoints from current image
      vector<cv::KeyPoint> keypoints; // create empty feature list for current image
      
      //// STUDENT ASSIGNMENT
      //// TASK MP.2 -> add the following keypoint detectors in file
      //// matching2D.cpp and enable
      ///  string-based selection based on detectorType
      //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
      
      // keypoints descriptor matrix (moved up from below)
      cv::Mat descriptors;
      double detectMS;
      if (detectorType.compare("SHITOMASI") == 0)
        {
          detectMS = detKeypointsShiTomasi (keypoints, imgGray, false);
        }
      else if (detectorType.compare("HARRIS") == 0)
        {
          detectMS = detKeypointsHarris (keypoints, imgGray, false);
        }
      else if (detectorType.compare("BRISK") == 0)
        {
          detectMS = detKeypointsBRISK (keypoints, imgGray, false);
        }
      else if (detectorType.compare("FAST") == 0)
        {
          detectMS = detKeypointsFAST (keypoints, imgGray, false);
        }
      else if (detectorType.compare("AKAZE") == 0)
        {
          detectMS = detKeypointsAKAZE (keypoints, imgGray, descriptors, false);
        }
      else if (detectorType.compare("ORB") == 0)
        {
          detectMS = detKeypointsORB (keypoints, imgGray, descriptors, false);
        }
      else if (detectorType.compare("SIFT") == 0)
        {
          // SIFT does not compile/link on my PC!
          detectMS = detKeypointsSIFT (keypoints, imgGray, false);
        }
      else
        {
          cout << " ERROR! detector type is unknown!\n";
          exit (0);
          //...
        }

      //// EOF STUDENT ASSIGNMENT
      
      //// STUDENT ASSIGNMENT
      //// TASK MP.3 -> only keep keypoints on the preceding vehicle
      
      // only keep keypoints on the preceding vehicle
      bool bFocusOnVehicle = true;
      cv::Rect vehicleRect(535, 180, 180, 150);
      if (bFocusOnVehicle)
        {
          vector<cv::KeyPoint> keepers;
          for (int n=0; n<keypoints.size(); n++)
            {
              int x = keypoints[n].pt.x;
              int y = keypoints[n].pt.y;
              
              if ( vehicleRect.x < x && x < vehicleRect.x + vehicleRect.width  &&
                   vehicleRect.y < y && y < vehicleRect.y + vehicleRect.height )
                {
                  // This is inside the rectangle
                  keepers.push_back (keypoints[n]);
                }
              // ...
            }
          keypoints.swap (keepers); // replace 'keypoints' with 'keepers'

          // Distribution of keypoint sizes
          std::sort (keypoints.begin(), keypoints.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2) {
              return (kp1.size < kp2.size); 
            });

          stringstream sst;
          sst << "MP.7 Detector " << detectorType << " keypoints " << keepers.size() << " img " << imgNumber.str() 
              << " smallest " << keypoints.begin()->size << " biggest " << (keypoints.end()-1)->size << endl;
          outFID << sst.str ();
          
          cout << "MP.7 Detector " << detectorType << " keypoints " << keepers.size() << " img " << imgNumber.str() 
               << " smallest " << keypoints.begin()->size << " biggest " << (keypoints.end()-1)->size << endl;

          
        }
      //// EOF STUDENT ASSIGNMENT
      
      // optional : limit number of keypoints (helpful for debugging and learning)
      bool bLimitKpts = false;
      if (bLimitKpts)
        {
          int maxKeypoints = 50;
          
          if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
              keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
          cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
          cout << " NOTE: Keypoints have been limited!" << endl;
        }
      
      // push keypoints and descriptor for current frame to end of data buffer
      (dataBuffer.end() - 1)->keypoints = keypoints;
      cout << "#2 : DETECT KEYPOINTS done" << endl;
      
      /* EXTRACT KEYPOINT DESCRIPTORS */
      
      //// STUDENT ASSIGNMENT
      //// TASK MP.4 -> add the following descriptors in file
      ////              matching2D.cpp and enable
      ////              string-based selection based on descriptorType
      //// -> BRIEF, ORB, FREAK, AKAZE, SIFT, BRISK (provided)
      
      double extractMS = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
      cout << " ..Descriptor size = " << descriptors.size() << endl;
      
      stringstream sst;
      sst << "MP.9 Det " << detectorType << " Desc " << descriptorType
           << " img " << imgNumber.str() << " detectMS " << detectMS << " extractMS " << extractMS << " "
           << detectMS+extractMS << endl;
      outFID << sst.str ();      

      cout << "MP.9 Det " << detectorType << " Desc " << descriptorType
           << " img " << imgNumber.str() << ": detectMS " << detectMS << " extractMS " << extractMS << ": "
           << detectMS+extractMS << endl;

      //// EOF STUDENT ASSIGNMENT
      
      // push descriptors for current frame to end of data buffer
      (dataBuffer.end() - 1)->descriptors = descriptors;
      
      cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
      
      if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
          
          /* MATCH KEYPOINT DESCRIPTORS */
          
          vector<cv::DMatch> matches;
          // These are now defined inthe main scope
          //string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
          //string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
          
          // This is the 'norm' type
          string matcherNorm = "DES_BINARY"; // DES_BINARY, DES_HOG
          
          //// STUDENT ASSIGNMENT
          //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
          //// TASK MP.6 -> add KNN match selection and perform
          ////              descriptor distance ratio filtering
          ////              with t=0.8 in file matching2D.cpp
          
          matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                           (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                           matches, matcherNorm, matcherType, selectorType);
          cout << " Count of matches ' " << matches.size() << endl;
          //// EOF STUDENT ASSIGNMENT
          
          // store matches in current data frame
          (dataBuffer.end() - 1)->kptMatches = matches;
          
          cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
          float mRatio = matches.size() / ((float)keypoints.size());
          stringstream sst;
          sst << "MP.8 Det " << detectorType << " Desc " << descriptorType << " matches " << matches.size() 
               << " img " << imgNumber.str() << " note_keypts " << keypoints.size() << " " << mRatio << endl;
          outFID << sst.str ();

          cout << "MP.8 Det " << detectorType << " Desc " << descriptorType << " matches " << matches.size() 
               << " img " << imgNumber.str() << ": note keypts " << keypoints.size() << " (" << mRatio << ")" << endl;

          // visualize matches between current and previous image
          bVis = true;
          if (bVis)
            {
              cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
              cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                              (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                              matches, matchImg,
                              cv::Scalar::all(-1), cv::Scalar::all(-1),
                              vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
              
              string windowName = "Matching keypoints between two camera images";
              cv::namedWindow(windowName, 7);
              cv::imshow(windowName, matchImg);
              cout << "Press key to continue to next image" << endl;
              cv::waitKey(0); // wait for key to be pressed
            }
          bVis = false;
        }
      
    } // eof loop over all images

  outFID.close ();
  return 0;
}
