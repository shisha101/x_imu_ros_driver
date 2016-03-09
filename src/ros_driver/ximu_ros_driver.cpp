#include "ximuapi/enumerations.h"
#include "ximuapi/serialization/reader_base.h"
#include "ximuapi/serialization/writer_base.h"
#include <iostream>
#include <unistd.h>

// ROS includes
#include "serial/serial.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
//#ifndef NDEBUG
//#define NDEBUG
//#endif


class XimuROS : public ximu::ReaderBase { //:  public ximu::WriterBase
public:
  serial::Serial _serial_obj;
  ros::Publisher _imu_data_pub;
  ros::Publisher _mag_data_pub;

  XimuROS(ros::NodeHandle nh, serial::Timeout timeout = serial::Timeout::simpleTimeout(5000))
  {
    // default values
    const std::string imu_pub_tn_default = "/sensor/imu/xIMU/data";
    const std::string mag_pub_tn_default = "/sensor/imu/xIMU/magfield_msg";
    const std::string port_default = "/dev/ttyUSB0";
    const std::string frame_id_default = "imu_xIMU_position";
    const u_int32_t baude_rate_default = 115200; // should not be changed
    // read ros params
    ros::param::param<std::string>("~filterd_imu_data_topic_name", _imu_pub_tn, imu_pub_tn_default);
    ros::param::param<std::string>("~magnetic_field_msg_data_topic_name", mag_pub_tn, mag_pub_tn_default);
    ros::param::param<std::string>("~port", _port, port_default);
    ros::param::param<std::string>("~frame_name", _frame_id, frame_id_default);
    _baude_rate = baude_rate_default;

    _serial_obj.setBaudrate(_baude_rate);
    _serial_obj.setPort(_port);
    _serial_obj.setTimeout(timeout);
    std::cout << "Port : " << _serial_obj.getPort() << "\n";
    std::cout << "Baudrate : " << _serial_obj.getBaudrate() << "\n";
    std::cout << "Frame id : " << _frame_id << "\n";
    // set params that will not change with time
    _mag_msg.header.frame_id = _frame_id;
    _imu_msg.header.frame_id = _frame_id;
    // create publishers
    _imu_data_pub = nh.advertise<sensor_msgs::Imu> (_imu_pub_tn, 1);
    _mag_data_pub = nh.advertise<sensor_msgs::MagneticField>(mag_pub_tn, 1);
  }

  virtual void recievedQuaternionData(ximu::QuaternionData& q) {
    _imu_msg.orientation.w = q.w();
    _imu_msg.orientation.x = q.x();
    _imu_msg.orientation.y = q.y();
    _imu_msg.orientation.z = q.z();
//    _imu_data_pub.publish(_imu_msg);
#ifdef NDEBUG
    std::cout << "Quaternion recieved" << std::endl;
    std::printf("the Quaternion values are : w %7.3f   x %7.3f   y %7.3f   z %7.3f \n", q.w(), q.x(), q.y(), q.z());
#endif
  }

  virtual void recievedCalInertialAndMagneticData(ximu::CalInertialAndMagneticData& data){

    ximu::Vector3f acc_data = data.accelerometer();
    ximu::Vector3f gyro_data = data.gyroscope();
    ximu::Vector3f mag_data = data.magnetometer();
    // accelerometer data
    _imu_msg.linear_acceleration.x = acc_data.x();
    _imu_msg.linear_acceleration.y = acc_data.y();
    _imu_msg.linear_acceleration.z = acc_data.z();

    // gyroscope data
    _imu_msg.angular_velocity.x = gyro_data.x() * _deg_to_rad;
    _imu_msg.angular_velocity.y = gyro_data.y() * _deg_to_rad;
    _imu_msg.angular_velocity.z = gyro_data.z() * _deg_to_rad;

    // Magetometer Data
    _mag_msg.magnetic_field.x = mag_data.x();
    _mag_msg.magnetic_field.y = mag_data.y();
    _mag_msg.magnetic_field.z = mag_data.z();

    // publish data
    _mag_msg.header.stamp = ros::Time::now();
    _imu_msg.header.stamp = ros::Time::now();
    _mag_data_pub.publish(_mag_msg);
    _imu_data_pub.publish(_imu_msg);

#ifdef NDEBUG
    std::printf("the acc values are x %7.3f   y %7.3f   z %7.3f \n", acc_data.x(), acc_data.y(),acc_data.z());
    std::printf("the gyro values are x %7.3f   y %7.3f   z %7.3f \n", gyro_data.x(), gyro_data.y(),gyro_data.z());
    std::printf("the mag values are x %7.3f   y %7.3f   z %7.3f \n", mag_data.x(), mag_data.y(),mag_data.z());
#endif
  }

  int run_Ximu_ros_driver(int num_retries=10, int sleep_s=1)
  {
    ros::Rate loop_sleep_duration(1/sleep_s);
    open_serial_port();
    int retries = 0;
    while(ros::ok() && !_serial_obj.isOpen() && retries < num_retries) {
      if(!_serial_obj.isOpen())
      {
        std::cout << "serial port is closed, attempting to open" << std::endl;
        open_serial_port();
      }
      else
      {
        std::cout << "serial port is open" << std::endl;
        break;
      }
      loop_sleep_duration.sleep();
      //usleep(3000 * 1000 * 0.1);
      retries++;
    }

    if(should_quit(retries, num_retries)){
      return -1;
    }
    retries = 0;

    while (_serial_obj.available() < 4 && ros::ok() && retries < num_retries)
    {
      std::cout << "not enough chars on buffer, waiting ..." << std::endl;
      loop_sleep_duration.sleep();
    }

    if(should_quit(retries, num_retries)){
      return -1;
    }
    retries = 0;

  while(ros::ok()){
      std::vector<u_int8_t> serial_in;
      try{
      size_t num_chars_read = _serial_obj.read(serial_in, 1);
      this->fill(serial_in.begin(), serial_in.end()); // fill the buffer of the c++ api used for decoding
      read(); // decode data from c++ api buffer and call appropriate decoding fucntion and in turn virtual functions
      }
      catch(serial::SerialException& e)
      {
        std::cout << "SerialException exception" << std::endl;
        _serial_obj.close();
        if (!_serial_obj.isOpen()){
        open_serial_port();
        }
        else{
          ros::spinOnce();
          loop_sleep_duration.sleep();
        }
      }
      catch(serial::PortNotOpenedException& e)
      {
        std::cout << "port not open exception" << std::endl;
        if (!_serial_obj.isOpen()){
        open_serial_port();
        }
        else{
          ros::spinOnce();
          loop_sleep_duration.sleep();
        }
      }

#ifdef NDEBUG
      std::cout << "the numbers of chars read on the serail read command is/are: " << num_chars_read << std::endl;
#endif

  }
  }
  void open_serial_port()
  {
    try {
      _serial_obj.open();
    }
    catch (serial::SerialException& e) {
      std::cout << e.what() << std::endl;
      std::cout << "serial open has thrown an exception make sure the device is connected"<< std::endl;
    }
    catch (serial::IOException&e ) {
      std::cout << e.what() << std::endl;
      std::cout << "serial open has thrown an exception make sure the device is connected"<< std::endl;
    }
  }

  bool should_quit(int retries, int num_retries){
    if (retries >= num_retries){
      std::cout << "number of retries exceded quitting xIMU driver" << std::endl;
      return true; // quit driver
    }
    return false;
  }

  // getters and setters
  std::string getImu_pub_tn() const
  {
  return _imu_pub_tn;
  }
  void setImu_pub_tn(const std::string & value)
  {
  _imu_pub_tn = value;
  }
  std::string getMag_pub_tn() const
  {
  return mag_pub_tn;
  }
  void setMag_pub_tn(const std::string & value)
  {
  mag_pub_tn = value;
  }
  std::string getPort_default() const
  {
  return _port;
  }
  void setPort_default(const std::string & value)
  {
  _port = value;
  }
  std::string getFrame_id() const
  {
  return _frame_id;
  }
  void setFrame_id(const std::string & value)
  {
  _frame_id = value;
  }
  u_int32_t getBaude_rate() const
  {
  return _baude_rate;
  }
  void setBaude_rate(const u_int32_t & value)
  {
  _baude_rate = value;
  }

private:
  sensor_msgs::Imu _imu_msg;
  sensor_msgs::MagneticField _mag_msg;
  std::string _imu_pub_tn;
  std::string mag_pub_tn;
  std::string _port;
  std::string _frame_id;
  u_int32_t _baude_rate;
  const double _deg_to_rad = M_PI/180;
};


void run(std::string port, u_int32_t baud_rate)
{
  std::cout << "in Run" << std::endl;
  serial::Timeout time_out = serial::Timeout::simpleTimeout(5000);
  serial::Serial serial_obj;
  serial_obj.setBaudrate(9600);
  serial_obj.setPort(port);
  serial_obj.setTimeout(time_out);
  uint num_chars_buffer_wait = 10;
  try
  {
    serial_obj.open();
  }
  catch (serial::SerialException& e)
  {
    std::cout << e.what() << std::endl;
  }
  catch (serial::IOException&e )
  {
    std::cout << e.what() << std::endl;
    std::cout << "serial open has thrown an exception make sure the device is connected"<< std::endl;
  }

//  std::cout << serial_obj.getBaudrate() << std::endl;
    for(int i=0; i<20; i++)
    {
      std::cout << "iteration i" << std::endl;
      if(!serial_obj.isOpen())
      {
        std::cout << "serial port is closed" << std::endl;
      }
      else
      {
        std::cout << "serial port is open" << std::endl;
      }
      usleep(1000 * 1000 * 0.1);
    }

    while (serial_obj.available() == 0)
    {
      std::cout << "no chars on buffer, waiting ..." << std::endl;
    }
    std::cout << "The number of chars are " << serial_obj.available() << std::endl;
    serial_obj.close();
    if(!serial_obj.isOpen())
    {
      std::cout << "serial port is closed" << std::endl;
    }
    else
    {
      std::cout << "serial port is open" << std::endl;
    }
}

main(int argc, char** argv)
{
  ros::init(argc, argv, "ximu_driver");
  ros::NodeHandle nh;       // currently not being used
  ros::NodeHandle pnh("~"); // currently not being used
  XimuROS x_io_sensor_1(nh);
  x_io_sensor_1.run_Ximu_ros_driver();
}







