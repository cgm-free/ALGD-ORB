#include "ORB_modify.h"

ORB_modify::ORB_modify(const string &strSettingPath)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);
    // cout<<"mDistCoef: " << mDistCoef<< endl;

//    mbf = fSettings["Camera.bf"];

    // cout << endl << "Camera Parameters: " << endl;
    // cout << "- fx: " << fx << endl;
    // cout << "- fy: " << fy << endl;
    // cout << "- cx: " << cx << endl;
    // cout << "- cy: " << cy << endl;
    // cout << "- k1: " << DistCoef.at<float>(0) << endl;
    // cout << "- k2: " << DistCoef.at<float>(1) << endl;
    // if(DistCoef.rows==5)
    //     cout << "- k3: " << DistCoef.at<float>(4) << endl;
    // cout << "- p1: " << DistCoef.at<float>(2) << endl;
    // cout << "- p2: " << DistCoef.at<float>(3) << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    // if(mbRGB)
    //     cout << "- color order: RGB (ignored if grayscale)" << endl;
    // else
    //     cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    // cout << endl  << "ORB Extractor Parameters: " << endl;
    // cout << "- Number of Features: " << nFeatures << endl;
    // cout << "- Scale Levels: " << nLevels << endl;
    // cout << "- Scale Factor: " << fScaleFactor << endl;
    // cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    // cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}

void ORB_modify::ORB_feature(cv::Mat &im)
{
    cv::Mat mImGray = im;
    // cv::Mat Image;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }
//     cout << "灰度值"<< int(mImGray.at<uchar>(Point(1, 1)))<< endl;
//     cout << "灰度值"<< int(mImGray.at<uchar>(100, 100))<< endl;
//
//    cv::imshow("mImGray", mImGray);
//    cv::imwrite("./result/mImGray.png", mImGray);

    (*mpORBextractor)(mImGray,cv::Mat(),mvKeys,mDescriptors);

    N = mvKeys.size();
    if(mvKeys.empty())
    {
        cout<<"no orb features?" <<endl;
        return;
    }
        
//    UndistortKeyPoints();//注意不是任何图片都是这个 TUM2.yaml参数!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //新增
//    cout<<"mvKeys.size(): " << mvKeys.size()<< endl;
//    drawKeypoints(im, mvKeys, im, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
//    cout<<"mvKeysUn.size(): " << mvKeysUn.size()<< endl;

//    cv::imshow("orb_feature", im);
//    cv::imwrite("./result/orb_modify.png", im);

}

// void ORB_modify::UndistortKeyPoints()
// {
//     if(mDistCoef.at<float>(0)==0.0)
//     {
//         mvKeysUn=mvKeys;
//         cout<<"如果第一个畸变参数为0，不需要矫正。第一个畸变参数k1是最重要的，一般不为0，为0的话，说明畸变参数都是0"<< endl;
//         return;
//     }
//     cout<<"如果畸变参数不为0，用OpenCV函数进行畸变矫正"<< endl;
//     // Fill matrix with points
//     cv::Mat mat(N,2,CV_32F);
//     for(int i=0; i<N; i++)
//     {
//         mat.at<float>(i,0)=mvKeys[i].pt.x;
//         mat.at<float>(i,1)=mvKeys[i].pt.y;
//     }

//     // Undistort points
//     mat=mat.reshape(2);
//     cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
//     mat=mat.reshape(1);

//     // Fill undistorted keypoint vector
//     mvKeysUn.resize(N);
//     for(int i=0; i<N; i++)
//     {
//         cv::KeyPoint kp = mvKeys[i];
//         kp.pt.x=mat.at<float>(i,0);
//         kp.pt.y=mat.at<float>(i,1);
//         mvKeysUn[i]=kp;
//     }
// }