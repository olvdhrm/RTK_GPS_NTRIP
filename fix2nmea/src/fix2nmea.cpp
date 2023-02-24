#include <chrono>

#include "nmea_msgs/msg/sentence.h"
#include "sensor_msgs/msg/nav_sat_fix.h"
#include "ublox_msgs/msg/nav_posllh.h"

#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_msgs/msg/nav_posllh.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

using std::placeholders::_1;

class Transformer : public rclcpp::Node
{
 public:
  Transformer() 
  : Node("fix2nmea")
  {
   
	//Publishes nmea sentences to topic /ntrip_client/nmea
	nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("ntrip_client/nmea", 10);  
	
      	
      	//Subscribes to obtain NavSatFix messages from topic /ublox_gps_node/fix
      	navsatfix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("ublox_gps_node/fix", 10, std::bind(&Transformer::receiveNavSatFix, this, _1));
      	//navhpposllh_sub_ = this->create_subscription<ublox_msgs::msg::NavHPPOSLLH>("ublox_gps_node/fix", 10, std::bind(&Transformer::receiveNavHpPosLlh, this, _1));
  }
  
  private:
  
    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_sub_;
    //rclcpp::Subscription<ublox_msgs::msg::NavPOSLLH>::SharedPtr navhpposllh_sub_;

    void receiveNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg)
    {
  	//RCLCPP_INFO_ONCE("Received first NavSatFix message.");
  	// Conversion inspired from: https://answers.ros.org/question/377844/converting-sensors_msgsnavsatfix-to-nmea_msgssentence-messages/

  	char buf[255]; // Buffer for GPGGA sentence

  	// Time conversion
  	//auto time = navsat_msg->header.stamp.toBoost().time_of_day();	
  	boost::posix_time::ptime t = boost::posix_time::second_clock::universal_time();
  	auto time = t.time_of_day();
  	long int deci_seconds = navsat_msg->header.stamp.sec / 1e7;

  	// Latitude conversion
  	char lat_dir = navsat_msg->latitude < 0.0 ? 'S' : 'N';
  	int8_t lat_degs = navsat_msg->latitude;
  	double lat_mins = (navsat_msg->latitude - (double) lat_degs) * 60.0;
  	lat_degs = fabs(lat_degs); 
  	lat_mins = fabs(lat_mins);

  	// Longitude conversion
  	char lon_dir = navsat_msg->longitude < 0.0 ? 'W' : 'E';
  	int8_t lon_degs = navsat_msg->longitude;
  	double lon_mins = (navsat_msg->longitude - (double) lon_degs) * 60.0;

  	// Status conversion
  	int8_t status = navsat_msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX ? 4 : 3;

  	// Minimum number of satellites is service times 4.
  	int num_satellites = 0;
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_GPS) == sensor_msgs::msg::NavSatStatus::SERVICE_GPS) {
    		num_satellites += 4;
  	}
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS)
      	== sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS) {
    		num_satellites += 4;
  	}
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS)
      	== sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS) {
    		num_satellites += 4;
  	}
  	if ((navsat_msg->status.service & sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO)
      	== sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO) {
    		num_satellites += 4;
  	}

	// Longitude conversion
  	double alt_msl = navsat_msg->altitude;
  	alt_msl = alt_msl - 1.2; 
  	
	//double alt_geoid = 1.2;
	
  	uint8_t
      	len = sprintf(buf,
                    "$GNGGA,%02ld%02ld%02ld.%ld,%02d%04.7f,%c,%03d%05.7f,%c,%d,%d,0.63,%0.3f,M,1.2,M,",
                    time.hours(),
                    time.minutes(),
                    time.seconds(),
                    deci_seconds,
                    lat_degs,
                    lat_mins,
                    lat_dir,
                    lon_degs,
                    lon_mins,
                    lon_dir,
                    status,
                    num_satellites,
                    alt_msl
                    //alt_geoid
                    );

	// Calculate checksum of sentence and add it to the end of the sentence
	uint8_t checksum = 0;
  	for(int i = 1; i < len; i++)
  	{
    	checksum ^= buf[i];
  	}
  	sprintf(&buf[len], "*%02X",checksum);
  
  	nmea_msgs::msg::Sentence nmea_msg;
  	nmea_msg.header = navsat_msg->header;
  	nmea_msg.sentence = buf;
  	
  	nmea_pub_->publish(nmea_msg);
    }
    
    
    

};



int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transformer>());
    rclcpp::shutdown();
    
    return 0;
  }
