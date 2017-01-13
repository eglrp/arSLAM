#include <iostream>
#include <fstream>

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>

//
#include <opencv2/opencv_modules.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Quaterniond angle2Quan(double x,double y,double z)
{
    double qx,qy,qz,qw;

    double sx(sin(x/2.0)),sy(sin(y/2.0)),sz(sin(z/2.0));
    double cx(cos(x/2.0)),cy(cos(y/2.0)),cz(cos(z/2.0));

    qx = sy*sz*cx+cy*cz*sx;
    qy = sy*cz*cx+cy*sz*sx;
    qz = cy*sz*cx-sy*cz*sx;
    qw = cy*cz*cx-sy*sz*sx;

    return Eigen::Quaterniond(qw,qx,qy,qz);
}


int main() {
    std::cout.precision(20);
    std::cout << "Hello, World!" << std::endl;


    cv::VideoCapture cap("/dev/video0");
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_OPENNI_QVGA_60HZ, 60.0);

    cv::Mat in_img;
    cv::Mat *out_img_ptr;
    std::string win_name("debug");
    std::string threed_name("3ddebug");

    cv::namedWindow(const_cast<char *>(threed_name.c_str()));

    cv::namedWindow(const_cast<char *> (win_name.c_str()));

    cv::waitKey(10);

//    viz::Viz


    cv::aruco::Dictionary dic = (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));
    cv::Ptr<cv::aruco::Dictionary> dic_ptr(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));
    cv::Ptr<cv::aruco::DetectorParameters> detectorParameters_ptr(cv::aruco::DetectorParameters::create());
    detectorParameters_ptr->adaptiveThreshConstant = 1.0;


    cv::VideoWriter vr;
    vr.open("tmp.avi", 4, 30, cv::Size(1280, 720), true);



    /**
    * Read data to set intrinsc_matrix and distortion matrix.
    */
    cv::Mat intrinsic_matrix(3, 3, CV_32F);
    cv::Mat distortion_matrix(1, 5, CV_32F);

    uchar *dp = distortion_matrix.data;
    float *ddp;
    ddp = (float *) dp;


    uchar *di = intrinsic_matrix.data;
    float *ddi;
    ddi = (float *) di;


    std::fstream tfs("./data/distortion_matrix.txt");
    for (int k(0); k < 5; ++k) {
        double ttt;
        tfs >> ttt;

        std::cout << "tttt:" << ttt << std::endl;
        ddp[k] = ttt;


    }
    std::cout << distortion_matrix << std::endl;

//
    std::fstream ifs;
    ifs.open("./data/intrinsic_matrix.txt");
    std::cout << "isf:" << ifs.is_open() << std::endl;
    for (int ii(0); ii < 3; ++ii) {
        for (int jj(0); jj < 3; ++jj) {
            double d_tmp(0.0);
            ifs >> d_tmp;
            std::cout << d_tmp << "kkkk" << std::endl;

            ddi[ii * 3 + jj] = d_tmp;
        }
    }


    std::cout << "-----------------" << std::endl;
    std::cout << intrinsic_matrix << std::endl;

    /*
    * End read matrix.
    */


//    cap.set(CV_)




    while (cap.isOpened()) {

        cap >> in_img;

        vr << in_img;

//        std::cout << "rows:"<<in_img.rows<<"  cols: "<<in_img.cols  << std::endl;
        out_img_ptr = &in_img;
        std::vector<std::vector<cv::Point2f>> corner;
        std::vector<int> ids;


        cv::aruco::detectMarkers(in_img, dic_ptr, corner, ids, detectorParameters_ptr);

        if (ids.size() > 0) {

            cv::aruco::drawDetectedMarkers(in_img, corner, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            try {
                cv::aruco::estimatePoseSingleMarkers(corner,
                                                     201,
                                                     intrinsic_matrix,
                                                     distortion_matrix,
                                                     rvecs,
                                                     tvecs);
                /**
                * Test transform matrix.
                */
                int i10(-1), i11(-1);
                for (int i(0); i < ids.size(); ++i) {
                    if (ids[i] == 11) {
                        i11 = i;
//                        std::cout<< "FIND 11"<<std::endl;
                    }
                    if (ids[i] == 10) {
                        i10 = i;
//                        std::cout << "FIND 10" << std::endl;
                    }
                }
//                printf("i10:%d,i11:%d",i10,i11);
                if ((i11 >= 0) && (i10 >= 0)  ) {
                    std::cout << "in" << std::endl;

                    Eigen::Affine3d t10,t11;
//                   Eigen::AngleAxisd r_10(),r_11;
                    Eigen::Quaterniond q10,q11;
                    q10 = angle2Quan(rvecs[i10](0),rvecs[i10](1),rvecs[i10](2));
                    q11 = angle2Quan(rvecs[i11](0),rvecs[i11](1),rvecs[i11](2));


                    q10.normalize();
                    q11.normalize();

                    t10.rotate(q10);
                    t11.rotate(q11);

                    t10.pretranslate(Eigen::Vector3d(tvecs[i10](0),tvecs[i10](1),tvecs[i10](2)));
                    t11.pretranslate(Eigen::Vector3d(tvecs[i11](0),tvecs[i11](1),tvecs[i11](2)));

                    Eigen::Vector3d src(0,0,0);

                    Eigen::Vector3d target(0,0,0);

                    target = t10 * src;
                    std::cout << t10.matrix() << std::endl;
                    std::cout << t11.matrix() << std::endl;

                    std::cout << "in - target :" << target.transpose() << std::endl;
                    target = t11.inverse() * target;


                    std::cout << "target:" << target.transpose() << std::endl;


                }


                for (int i(0); i < rvecs.size(); ++i) {
                    //201 mm
//                    if (ids[i] == 11)
//                    {
//                        double sintheta(0.0);
//                        for(int z(0);z<rvecs[i].rows;++z)
//                        {
//                            std::cout << rvecs[i](z)*180.0/M_PI << "  ";
//                            sintheta += rvecs[i](z)*rvecs[i](z);
//                        }
//                        std::cout << rvecs[i] << std::endl;
//                        double omega(std::sqrt(1-sintheta));
//                        std::cout << "omega:"<<omega << std::endl;
//                        rvecs[i].
//                        std::cout << std::endl;
//                    }

//                        std::cout << rvecs[i] << "   " << tvecs[i] << std::endl;


                    cv::aruco::drawAxis(*out_img_ptr, intrinsic_matrix, distortion_matrix,
                                        rvecs[i], tvecs[i], 100.1);

                }
            } catch (cv::Exception a) {
                std::cout << "error in estimate pose:" <<
                          a.err << std::endl;

            }

//            std::cout << 1.2 << std::endl;


//            cv::aruco::drawAxis(in_img, intrinsic_matrix, distortion_matrix,)
//            cv::aruco::drawMarker(dic,ids[0],2,out_img);
        }


//        out_img = in_img;

        cv::imshow(win_name, in_img);
        cv::waitKey(10);

    }


    return 0;
}