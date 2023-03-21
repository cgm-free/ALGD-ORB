#include <iostream>
#include <string>

#include "gms_matcher.h"
#include "ORB_modify.h"
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"

using namespace std;
using namespace cv;
//extern double ORBextractor::ORBextractorDescriptorsTime;

//计算均匀度Uniformity
double ComputeUniformity(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints)
{
    //******************************************************************均匀性计算
    int KPLeftNum = 0, KPRightNum = 0;              //左右部分的关键点数量
    int KPTopNum = 0, KPDownNum = 0;                //上下部分的关键点数量
    int KP_Right_Top_Num = 0, KP_Left_Down_Num = 0; //右上,左下部分的关键点数量
    int KP_Left_Top_Num = 0, KP_Right_Down_Num = 0; //左上，右下部分的关键点数量
   int KPCentreNum = 0, KPPeripheryNum = 0;        //中心，外围部分的关键点数量
//******************************************************************均匀性计算
//**********************************************************************************

    const int minBorderX = 0;
    const int minBorderY = 0;
    const int maxBorderX = img.cols;
    const int maxBorderY = img.rows;
//     cout << "maxBorderX:" << img.cols << endl;
//     cout << "maxBorderY:" << img.rows << endl;
//   cout << "img.size:" << img.size << endl;

//**********************************************************************************

    for (std::vector<cv::KeyPoint>::iterator vit = keypoints.begin(); vit != keypoints.end(); vit++)
    {

        //****************************左右*************************************//
        // line(img, Point((maxBorderX - minBorderX) / 2, 0), Point((maxBorderX - minBorderX) / 2, maxBorderY), 8);
        // imshow("左右", img);
        if ((*vit).pt.x <= (maxBorderX - minBorderX) / 2) //左半部分的特征点数量
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 255, 120), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            // waitKey(0);//敲键盘关图片，别直接×
            KPLeftNum++;
        }
        else //右半部分的特征点数量
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 0, 255), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            // waitKey(0);//敲键盘关图片，别直接×
            KPRightNum++;
        }

        //****************************上下*************************************//
        // line(img, Point(0, (maxBorderY - minBorderY) / 2), Point(maxBorderX, (maxBorderY - minBorderY) / 2), 8);
        // imshow("上下", img);
        if ((*vit).pt.y <= (maxBorderY - minBorderY) / 2) //上半部分的特征点数量
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 255, 120), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            KPTopNum++;
        }
        else //下半部分的特征点数量
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 0, 255), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            KPDownNum++;
        }
        
        //****************************右上左下*************************************//
        // line(img, Point(0, 0), Point(maxBorderX, maxBorderY), 8);
        // imshow("右上左下", img);
        if (((*vit).pt.y - maxBorderY) * (maxBorderX - minBorderX) -
            ((*vit).pt.x - maxBorderX) * (maxBorderY - minBorderY) >=
            0) //右上半部分的特征点数量
            //这是对角线方程 (y - y2)*(x1 - x2) - (x - x2)*(y1 - y2)
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 255, 120), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            KP_Right_Top_Num++;
        }
        else //左下半部分的特征点数量
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 0, 255), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            KP_Left_Down_Num++;
        }

        //****************************右下左上*************************************//
        // line(img, Point(maxBorderX, 0), Point(0, maxBorderY), 8);
        // imshow("右下左上", img);
        if (((*vit).pt.y - maxBorderY) * (maxBorderX - minBorderX) -
            ((*vit).pt.x - minBorderX) * (minBorderY - maxBorderY) >
            0) //左上半部分的特征点数量
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 255, 120), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            // waitKey(0);//敲键盘关图片，别直接×
            KP_Left_Top_Num++;
        }
        else //右下半部分的特征点数量
        {
            // circle(img,(*vit).pt, 3, Scalar(0, 0, 255), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            KP_Right_Down_Num++;
        }
//  不要中心外围这部分的数据,原因:不能平分面积
        //****************************中心外围*************************************//
        float X1 =     (maxBorderX - minBorderX) / 6 - 8; //中心框的坐标X1
        float X2 = 5 * (maxBorderX - minBorderX) / 6 + 8; //中心框的坐标X2
        float Y1 =     (maxBorderY - minBorderY) / 6 - 8; //中心框的坐标Y1
        float Y2 = 5 * (maxBorderY - minBorderY) / 6 + 8; //中心框的坐标Y2 

        // line(img, Point(X1, Y1), Point(X2, Y1), 8);
        // line(img, Point(X1, Y1), Point(X1, Y2), 8);
        // line(img, Point(X2, Y1), Point(X2, Y2), 8);
        // line(img, Point(X1, Y2), Point(X2, Y2), 8);

        // imshow("中心外围", img);
       if (((*vit).pt.x >= X1) & ((*vit).pt.x <= X2) & ((*vit).pt.y >= Y1) & ((*vit).pt.y <= Y2) )//中心部分的特征点数量
       {
            // circle(img,(*vit).pt, 3, Scalar(0, 255, 120), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            KPCentreNum++;
       }
       else //外围部分的特征点数量
       {
            // circle(img,(*vit).pt, 3, Scalar(0, 0, 255), -1);//画点，其实就是实心圆
            // imshow("PointsinImage", img);
            KPPeripheryNum++;
       }
    }
    //***********************************去除中心外围,共8个区域***********************************************
//     int sum_ =
//             KPLeftNum + KPRightNum + KPTopNum + KPDownNum + KP_Right_Top_Num + KP_Left_Down_Num + KP_Left_Top_Num +
//             KP_Right_Down_Num ;
//     double mean_ = sum_ / 8;
//     double Variance = (pow((KPLeftNum - mean_),2)  + pow((KPRightNum - mean_),2) +
//                        pow((KPTopNum - mean_),2) + pow((KPDownNum - mean_) ,2) +
//                        pow((KP_Right_Top_Num - mean_) ,2) + pow((KP_Left_Down_Num - mean_) ,2) +
//                        pow((KP_Left_Top_Num - mean_) ,2)+ pow((KP_Right_Down_Num - mean_) ,2)) /8;
//     double Uniformity =  101*log(Variance);
//    cout << "左半部分的特征点数量:" << KPLeftNum << endl;
//    cout << "右半部分的特征点数量:" << KPRightNum << endl;
//    cout << "上半部分的特征点数量:" << KPTopNum << endl;
//    cout << "下半部分的特征点数量:" << KPDownNum << endl;
//    cout << "右上半部分的特征点数量:" << KP_Right_Top_Num << endl;
//    cout << "左下半部分的特征点数量:" << KP_Left_Down_Num << endl;
//    cout << "左上半部分的特征点数量:" << KP_Left_Top_Num << endl;
//    cout << "右下半部分的特征点数量:" << KP_Right_Down_Num << endl;
//    cout << "八个区域特征点数的平均值:" << mean_ << endl;
//    cout << "八个区域特征点数的方差:" << Variance << endl;
//    cout << "图像特征点均匀度:" << Uniformity << endl;
//     cout << endl;
//     return Uniformity;
    //***********************************加上中心外围,共10个区域***********************************************

   int sum_ =
           KPLeftNum + KPRightNum + KPTopNum + KPDownNum + KP_Right_Top_Num + KP_Left_Down_Num + KP_Left_Top_Num +
           KP_Right_Down_Num + KPCentreNum + KPPeripheryNum;
   double mean_ = sum_ / 10; //约等于500
   double Variance = (pow((KPLeftNum - mean_),2)  + pow((KPRightNum - mean_),2) +
                      pow((KPTopNum - mean_),2) + pow((KPDownNum - mean_) ,2) +
                      pow((KP_Right_Top_Num - mean_) ,2) + pow((KP_Left_Down_Num - mean_) ,2) +
                      pow((KP_Left_Top_Num - mean_) ,2)+ pow((KP_Right_Down_Num - mean_) ,2)  +
                      pow((KPCentreNum - mean_) ,2) + pow((KPPeripheryNum - mean_) ,2) ) /10;
   double Uniformity =  101*log10(Variance);
//    cout << "左半部分的特征点数量:" << KPLeftNum << endl;
//    cout << "右半部分的特征点数量:" << KPRightNum << endl;
//    cout << "上半部分的特征点数量:" << KPTopNum << endl;
//    cout << "下半部分的特征点数量:" << KPDownNum << endl;
//    cout << "右上半部分的特征点数量:" << KP_Right_Top_Num << endl;
//    cout << "左下半部分的特征点数量:" << KP_Left_Down_Num << endl;
//    cout << "左上半部分的特征点数量:" << KP_Left_Top_Num << endl;
//    cout << "右下半部分的特征点数量:" << KP_Right_Down_Num << endl;
//    cout << "中心部分的特征点数量:" << KPCentreNum << endl;
//    cout << "外围部分的特征点数量:" << KPPeripheryNum << endl;
//    cout << "十个区域特征点数的平均值:" << mean_ << endl;
//    cout << "十个区域特征点数的方差:" << Variance << endl;
//    cout << "图像特征点均匀度:" << Uniformity << endl;
//    cout << endl;
    return Uniformity;
}
//**********************************************************************************
//计算RANSAC后的RMSE
double calculate_RANSAC_inliers_RMSE(Mat& img1, Mat& img2, 
                                    vector<DMatch> & all_matches, 
                                    std::vector<cv::KeyPoint>  & leftmvKeysUn,
                                    std::vector<cv::KeyPoint>  & rightmvKeysUn,string &Argv4)
{
// Take only the matched points that will be used to calculate the
    // transformation between both images
    // TODO:只取匹配的点，用于计算两个图像之间的转换
    std::vector<cv::Point2d> matched_pts1, matched_pts2;
    for (cv::DMatch _match_ : all_matches)// TODO:不用描述子距离判断all_matches_filter
    {
        matched_pts1.push_back(leftmvKeysUn[_match_.queryIdx].pt);
        matched_pts2.push_back(rightmvKeysUn[_match_.trainIdx].pt);
    }
    // Find the homography that transforms a point in the first image to a point in the second image.
    cv::Mat inliers;
    cv::Mat H = cv::findHomography(matched_pts1, matched_pts2, cv::RANSAC, 3, inliers);
    // Print the number of inliers, that is, the number of points correctly
    // mapped by the transformation that we have estimated
    std::cout << "RANSAC去除错误匹配后的内点数: " << cv::sum(inliers)[0]
              << " ( 精度:" << (100.0f * cv::sum(inliers)[0] / all_matches.size()) << "% )" << std::endl;//cv::sum(inliers)[0] / all_matches.size())
//    cout<< "H：" <<  H <<endl;
    const double h11 = H.at<double>(0,0);
    const double h12 = H.at<double>(0,1);
    const double h13 = H.at<double>(0,2);
    const double h21 = H.at<double>(1,0);
    const double h22 = H.at<double>(1,1);
    const double h23 = H.at<double>(1,2);
    const double h31 = H.at<double>(2,0);
    const double h32 = H.at<double>(2,1);
    const double h33 = H.at<double>(2,2);
//        const float h11 = 1.0107879e+00;
//        const float h12 = 8.2814684e-03;
//        const float h13 = 1.8576800e+01;
//        const float h21 = -4.9128885e-03;
//        const float h22 = 1.0148779e+00 ;
//        const float h23 = -2.8851517e+01;
//        const float h31 = -1.9166087e-06;
//        const float h32 = 8.1537620e-06 ;
//        const float h33 = 1.0000000e+00;

    vector<DMatch> optimizeM;
    for(int i = 0; i < inliers.rows; i++)
    {
        if(inliers.at<bool>(i,0))
        {
            optimizeM.push_back(all_matches[i]);// TODO:不用描述子距离判断all_matches_filter
        }
    }

    std::vector<cv::Point2d> matched_pts3, matched_pts4;
    for (cv::DMatch matc1 : optimizeM)
    {
        matched_pts3.push_back(leftmvKeysUn[matc1.queryIdx].pt);
        matched_pts4.push_back(rightmvKeysUn[matc1.trainIdx].pt);
    }
    Mat result2;
    drawMatches(img1, leftmvKeysUn, img2, rightmvKeysUn, optimizeM, result2, Scalar(0, 255, 0), Scalar::all(-1));//Scalar::all(-1)

    imwrite("./result/ORB_RANSAC_matcher_"+Argv4+".png", result2);
    // imshow("ORB_RANSAC_matcher", result2);

    std::vector<Point2d> matched_pts2_ture_H;
//  "[x',y',1] " <<  H*[x,y,1]
    for (std::vector<cv::Point2d>::iterator vit = matched_pts3.begin(); vit != matched_pts3.end(); vit++)
    {

        double x1 = (*vit).x;//左图匹配的特征点的x坐标
        double y1 = (*vit).y;//左图匹配的特征点的y坐标
        // Reprojection error in second image
        // x1in2 = H21*x1
        Point2d temp;
        temp.x =(h11*x1 + h12*y1 + h13)/(h31*x1 + h32*y1 + h33);//左图匹配的特征点的x坐标 经过单应矩阵H  变换到右图的x坐标
        temp.y =(h21*x1 + h22*y1 + h23)/(h31*x1 + h32*y1 + h33);//左图匹配的特征点的y坐标 经过单应矩阵H  变换到右图的y坐标
        matched_pts2_ture_H.push_back(temp);
    }

    std::vector<cv::Point2d>::iterator vit = matched_pts2_ture_H.begin();
    std::vector<cv::Point2d>::iterator vit2 = matched_pts4.begin();

    double fsum=0,RMSE=0;
    int sum=0;
    while(vit != matched_pts2_ture_H.end())
    {
//        cout<< "vit " <<  (*vit) <<endl;//左图匹配的点坐标[x1,y1]经过单应矩阵H变换到右图的点坐标[x2,y2]
//        cout<< "vit2 " <<  (*vit2) <<endl;//测量值
        fsum += pow(((*vit).x-(*vit2).x),2) + pow(((*vit).y-(*vit2).y),2);
        vit++;
        vit2++;
        sum++;
    }
    RMSE = sqrt(fsum/matched_pts4.size());
    cout<< "RANSAC去除错误匹配后,配对点(内点)的均方根误差RMSE: " <<  RMSE <<endl;
    cout<<endl;
    return  RMSE;
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cout << "usage: ./orb_matcher path_to_settings path_to_image1 path_to_image2 oxford1x" << endl;
        return 1;
    }
    cout<<"***********************************************************";
    cout<<argv[4]<<endl;
    string Argv4;
    Argv4 =argv[4];
    // Check settings file
    const string strSettingsFile = argv[1];
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    cv::Mat img1 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    cv::Mat img2 = imread(argv[3], CV_LOAD_IMAGE_COLOR);

    ORB_modify ORB_left(strSettingsFile);  //读取yaml文件
    ORB_modify ORB_right(strSettingsFile); //读取yaml文件
    ORB_left.ORB_feature(img1);            //提取左图特征  因为不是TUM2数据集,所以其他图片不要用mvKeysUn这个,用未矫正的mvKeys!!!!!
    ORB_right.ORB_feature(img2);           //提取右图特征  因为不是TUM2数据集,所以其他图片不要用mvKeysUn这个,用未矫正的mvKeys!!!!!

    Mat img1_ORB_feature;
    Mat img2_ORB_feature;


    drawKeypoints(img1, ORB_left.mvKeys, img1_ORB_feature, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
    drawKeypoints(img2, ORB_right.mvKeys, img2_ORB_feature, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

    // cv::imshow("img1_ORB_feature", img1_ORB_feature);//不显示图片的特征点
    // cv::imwrite("./result/img1_ORB_feature.png", img1_ORB_feature);//不保存图片的特征点
//    cout << "img1 Get total " << ORB_left.mvKeys.size() << " keypoints." << endl;

    // cv::imshow("img2_ORB_feature", img2_ORB_feature);//不显示图片的特征点
    // cv::imwrite("./result/img2_ORB_feature.png", img2_ORB_feature);//不保存图片的特征点
//    cout << "img2 Get total " << ORB_right.mvKeys.size() << " keypoints." << endl;

//***************************调用特征点均匀度函数ComputeUniformity*************************************************
//   cvtColor(img1,img1,CV_RGB2GRAY);
//   cvtColor(img2,img2,CV_RGB2GRAY);
    double leftUniformity = ComputeUniformity(img1, ORB_left.mvKeys);
    double rightUniformity = ComputeUniformity(img2, ORB_right.mvKeys);
    cout << "左图特征点均匀度:  "<< leftUniformity  << endl;
    cout << "右图特征点均匀度:  "<< rightUniformity <<endl;

//****************************暴力匹配:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离************************************************
    // // BFMatcher
    // BFMatcher matcher(NORM_HAMMING,true);
    // vector<DMatch> all_matches_BF, gms_matches;
    // matcher.match(ORB_left.mDescriptors, ORB_right.mDescriptors, all_matches_BF); //输出结果all_matches
    // Mat all_matches_img;
    // drawMatches(img1, ORB_left.mvKeys, img2, ORB_right.mvKeys, all_matches_BF, all_matches_img, Scalar(0, 255, 0), Scalar::all(-1));
    // imwrite("./result/ORB_BFmatcher.png", all_matches_img);
    // imshow("ORB_BFmatcher", all_matches_img);
    // cout << "图1获得特征点数 " << ORB_left.mvKeys.size() << " keypoints." << endl;
    // cout << "图2获得特征点数 " << ORB_right.mvKeys.size() << " keypoints." << endl;
    // std::cout << "暴力匹配后个数: " << all_matches_BF.size() << std::endl;

// ****************************Hamming 距离筛选匹配:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离************************************************
//*** 匹配耗时
std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
   vector<DMatch> all_matches_BF, gms_matches,all_matches_distance_filter;
   Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
   matcher->match(ORB_left.mDescriptors, ORB_right.mDescriptors, all_matches_BF); //输出结果all_matches

   cout << "图1获得特征点数 " << ORB_left.mvKeys.size() << " keypoints." << endl;
   cout << "图2获得特征点数 " << ORB_right.mvKeys.size() << " keypoints." << endl;

//***********************匹配点对筛选(人为设计)**********************************************
   double min_dist = 10000, max_dist = 0;

   //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
   for (int i = 0; i < all_matches_BF.size(); i++) {
       double dist = all_matches_BF[i].distance;
       if (dist < min_dist) min_dist = dist;
       if (dist > max_dist) max_dist = dist;
   }

//    printf("BRIEF描述子-- Max dist : %f \n", max_dist);
//    printf("BRIEF描述子-- Min dist : %f \n", min_dist);

   //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限
   for (int i = 0; i < all_matches_BF.size(); i++) {
       if (all_matches_BF[i].distance <= max(2 * min_dist, 30.0))//增加特征点数或减少特征点数再这里调整
       {
           all_matches_distance_filter.push_back(all_matches_BF[i]);
       }
   }
std::chrono::steady_clock::time_point t22 = std::chrono::steady_clock::now();
double ORBmatchTime= std::chrono::duration_cast<std::chrono::duration<double> >(t22 - t11).count();
cout << "匹配耗时 :" << ORBmatchTime << endl;
   printf("ORB-- All matches : %d \n", (int) all_matches_BF.size());
//    printf("ORB-- Descriptors max distance : %d \n", (int) max(2 * min_dist, 30.0));
   printf("ORB-- Optimized matching : %d \n", (int) all_matches_distance_filter.size());
   Mat result;
   drawMatches(img1, ORB_left.mvKeys, img2, ORB_right.mvKeys, all_matches_distance_filter, result, Scalar(0, 255, 0), Scalar(0, 255, 0));///Scalar::all(-1)
//    imwrite("./result/ORB_matcher_distance_"+Argv4+".png", result); 
//    imshow("ORB_matcher_distance", result);

//*****************************************计算RMSE********参考BEBLID算法和SIFTS算法*******************************
//SIFT算法详解(附有完整代码)https://blog.csdn.net/weixin_47156401/article/details/122367593?spm=1001.2014.3001.5502
//***************************************************************************************************************
//TODO     计算经过RANSAC后的内点的均方根误差RMSE
    // calculate_RANSAC_inliers_RMSE(img1, img2, all_matches_BF, ORB_left.mvKeys, ORB_right.mvKeys);//暴力匹配筛选的
   calculate_RANSAC_inliers_RMSE(img1, img2, all_matches_distance_filter, ORB_left.mvKeys, ORB_right.mvKeys,Argv4);//Hamming距离阈值匹配筛选的

//*****************************************计算RMSE**************************************************************
//***************************************************************************************************************















    // GMS filter
    std::vector<bool> vbInliers;
    gms_matcher gms(ORB_left.mvKeys, img1.size(), ORB_right.mvKeys, img2.size(), all_matches_BF);
    // cout<< img1.size() <<endl;
    int num_inliers = gms.GetInlierMask(vbInliers, false, false);
//    cout << "With GMS Get total " << num_inliers << " matches." << endl;

    // collect matches
    for (size_t i = 0; i < vbInliers.size(); ++i)
    {
        if (vbInliers[i] == true)
        {
            gms_matches.push_back(all_matches_BF[i]);
        }
    }

    // draw matching
    Mat show = gms.DrawInlier(img1, img2, ORB_left.mvKeys, ORB_right.mvKeys, gms_matches, 2);

//    imwrite("./result/ORB_matcher_with_GMS.png", show);
//    imshow("ORB_matcher_with_GMS", show);

    cv::waitKey(0);
    return 0;
}