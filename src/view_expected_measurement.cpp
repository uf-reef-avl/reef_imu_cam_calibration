//
// Created by prashant on 9/13/19.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <opencv2/core.hpp>

#include <boost/bind.hpp>

#include <cv_bridge/cv_bridge.h>
#include <camera_imu_calib/ExpectedMeasurement.h>

class ExpectedMeas{

private:

    cv::Mat inImage;

    image_transport::Publisher image_pub;

    image_transport::ImageTransport it_;

    ros::NodeHandle nh_;
    image_transport::SubscriberFilter image_sub;
    message_filters::Subscriber<camera_imu_calib::ExpectedMeasurement> expected_subs;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, camera_imu_calib::ExpectedMeasurement> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> synchronizer_;


public:

    ExpectedMeas():
        nh_(""),
        it_(nh_),
        image_sub(it_, "camera/rgb/image_raw",1),
        expected_subs(nh_,"expected_measurements",1),
        synchronizer_(SyncPolicy(5), image_sub, expected_subs)
    {
        synchronizer_.registerCallback(boost::bind(&ExpectedMeas::callback, this , _1,_2));
        image_pub = it_.advertise("residual_image",1);

    }

    void callback(const sensor_msgs::ImageConstPtr& image, const camera_imu_calib::ExpectedMeasurementConstPtr& exp_msg){

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        for(unsigned int i = 0; i<exp_msg->expected_measurement.size();i++){
            cv::Point2d expected_point(exp_msg->expected_measurement.at(2*i), exp_msg->expected_measurement.at(2*i + 1) );
            cv::circle(inImage, expected_point, 2, CV_RGB(255,0,0), -1);
            cv::Point2d pixel_point(exp_msg->pixel_measurement.at(2*i), exp_msg->pixel_measurement.at(2*i + 1) );
            cv::circle(inImage, pixel_point, 2, CV_RGB(0,255,0), -1);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = image->header;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
    }



};

int main(int argc, char **argv) {
    ros::init(argc, argv, "show_res");
    ExpectedMeas obj;

    ros::spin();
}

