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



class XimuROS : public ximu::ReaderBase { //:  public ximu::WriterBase
public:
  serial::Serial _serial_obj;
  XimuROS(const std::string port=_port_name_default, std::string frame_id = _frame_id_default,serial::Timeout timeout = serial::Timeout::simpleTimeout(5000))
  {
    _serial_obj.setBaudrate(_baude_rate);
    _serial_obj.setPort(port);
    _serial_obj.setTimeout(timeout);
    std::cout << "Port :" << _serial_obj.getPort() << "\n";
    std::cout << "Baudrate :" << _serial_obj.getBaudrate() << "\n";
    // set params that will not change with time
    _mag_msg.header.frame_id = frame_id;
    _imu_msg.header.frame_id = frame_id;
  }
  virtual void recievedQuaternionData(ximu::QuaternionData& q) {
    std::cout << "Quaternion recieved" << std::endl;
    std::vector<float> Q_values = q.values();
    std::cout << "the Quaternion values are : w: "<<Q_values[0]<<"x: " <<Q_values[1] <<"y: "<<Q_values[2] <<"z: "<<Q_values[3] <<std::endl;
  }
  virtual void recievedCalInertialAndMagneticData(ximu::CalInertialAndMagneticData& data){
    std::cout << "Inertial and Magnetic" << std::endl;
  }

  int run_Ximu_ROS_test()
  {
    uint _num_chars_buffer_wait = 10;
    try {
      _serial_obj.open();
    }
    catch (serial::SerialException& e) {
      std::cout << e.what() << std::endl;
    }
    catch (serial::IOException&e ) {
      std::cout << e.what() << std::endl;
      std::cout << "serial open has thrown an exception make sure the device is connected"<< std::endl;
    }
    for(int i=0; i<20; i++) {
      std::cout << "iteration i" << std::endl;
      if(!_serial_obj.isOpen())
      {
        std::cout << "serial port is closed" << std::endl;
      }
      else
      {
        std::cout << "serial port is open" << std::endl;
        break;
      }
      usleep(1000 * 1000 * 0.1);
    }

    while (_serial_obj.available() == 0)
    {
      std::cout << "no chars on buffer, waiting ..." << std::endl;
      usleep(1000 * 1000 * 0.1);
    }
    while (_serial_obj.available() < 4)
    {
      std::cout << "not enough chars on buffer, waiting ..." << std::endl;
      usleep(1000 * 1000 * 0.1);
    }

    std::cout << "xIMU is reading data ..." << std::endl;
  while(true){ // ros::ok()
//      std::string serial_in = _serial_obj.readline(30);
      std::vector<u_int8_t> serial_in;
      _serial_obj.read(serial_in, 1);
      this->fill(serial_in.begin(), serial_in.end());
//      std::cout << "serial in" << std::string(serial_in.begin(), serial_in.end());

      read();
  }
  }

  sensor_msgs::Imu get_imu_msg() {
    return _imu_msg;
  }
  sensor_msgs::MagneticField get_mag_msg() {
    return _mag_msg;
  }

private:
  const std::string _port_name_default = "/dev/ttyUSB0";
  const std::string _frame_id_default = "xIMU";
  const u_int32_t _baude_rate = 115200;
  const std::string imu_pub_topic_name_default = "";
  const std::string mag_pub_topic_name_default = "";
  sensor_msgs::Imu _imu_msg;
  sensor_msgs::MagneticField _mag_msg;

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
  const std::string default_port_name = "/dev/ttyUSB0";
  const u_int32_t baude_rate = 115200;
  std::cout << "Hello World" << std::endl;

//  serial::Serial serial_obj("/dev/ttyUSB0", 9600);
//  std::cout << "iteration i" << std::endl;
//  for(int i=0; i<20; i++)
//  {
//    std::cout << "iteration i" << std::endl;
//    if(!serial_obj.isOpen())
//    {
//      std::cout << "serial port is closed" << std::endl;
//    }
//  }
//  return 0;

//  run(port_name, baude_rate);
  XimuROS x_io_sensor_1(port_name, baude_rate);
  x_io_sensor_1.run_Ximu_ROS_test();
}

