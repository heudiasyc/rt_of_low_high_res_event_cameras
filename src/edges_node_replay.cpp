/**
 * edges_node_replay.cpp
 * This code corresponds to the ROS node which goal is to process the events from a .bag or .dat
 * file, and to compute the edge images from them.
 * This is the first node of the pipeline architecture.
 */

#include "rt_of_low_high_res_event_cameras/defines.hpp"
#include "rt_of_low_high_res_event_cameras/utils.hpp"

#include "rt_of_low_high_res_event_cameras/edges_cpu.hpp"

#include <dvs_msgs/EventArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>

#if METAVISION_SDK_AVAILABLE
#include <metavision/sdk/base/events/event2d.h>
#endif

#if HDF5_SDK_AVAILABLE
#include <H5Cpp.h>
#endif

#if FPS_MEASUREMENT
#include <fstream>
ofstream myfile;
#endif


/**
 * Global variables definition
 */

// The buffer of events, used for storing incomming events
deque<dvs_msgs::Event> evt_buffer;

// Input topic, if a .bag is used as the input
string topic;

// Height and width of the event sensor
int height, width;

// ROS publisher, used to send the edge image once computed
ros::Publisher edges_pub;


/**
 * \brief Processes all the events from the given .bag file to create the corresponding edge
 * images, which are then sent for further processing.
 *
 * \param bag_path The path to the input .bag file
 * \param calib_path The path to the input calibration file (empty = no calib file)
 * \param accumul_window The time window Δt for accumulating the events (in ms)
 */
void process_events_from_bag(const string& bag_path, const string& calib_path, int accumul_window)
{
  // Opening the rosbag
  rosbag::Bag inbag;
  inbag.open(bag_path, rosbag::bagmode::Read);
  if(!inbag.isOpen()) {
    ROS_ERROR_STREAM("Could not open rosbag with path '" << bag_path << "', exiting!");
    exit(EXIT_FAILURE);
  }

  // Computing the undistortion matrix, used if a calibration file of the camera is given
  Mat undistortion_matrix;
  if(!calib_path.empty()) {
    undistortion_matrix = init_undistort_mat(calib_path, height, width);
  }

  // Extracting a window Δt of events from the rosbag might take less time than Δt
  // Therefore, we must track the computation time we take to create each edge image, so that the
  // publish rate of this "replay" module is the same as the one of the "live" module
  chrono::system_clock::time_point start_delta_t = chrono::system_clock::now();

  // For each message of the given topic...
  for(rosbag::MessageInstance const m: rosbag::View(inbag)) {
    if(m.getTopic() == topic) {
      // ... we add its events to the buffer...
      dvs_msgs::EventArrayPtr evt_msg = m.instantiate<dvs_msgs::EventArray>();
      evt_buffer.insert(evt_buffer.end(), std::begin(evt_msg->events), std::end(evt_msg->events));

      // ... and we check that they are in the correct order.
      if(evt_buffer.back().ts < evt_buffer.front().ts) {
        ROS_WARN("Badly ordered events timestamps, cleaning the queue");
        evt_buffer.clear();
      } else {
        // If this is the case, and if enough events were received to surpass the accumulation
        // window Δt, we extract exactly the correct number of events to create an edge image of
        // time length Δt
        while(
          !evt_buffer.empty() && evt_buffer.back().ts - evt_buffer.front().ts >=
          ros::Duration().fromSec(accumul_window/1000.))
        {
          vector<dvs_msgs::Event> evts_to_process;
          for(const dvs_msgs::Event& e : evt_buffer) {
            evts_to_process.push_back(e);
            evt_buffer.pop_front();
            if(evts_to_process.back().ts - evts_to_process.front().ts
              >= ros::Duration().fromSec(accumul_window/1000.))
            {
              break;
            }
          }

#if FPS_MEASUREMENT
          ros::Time begin = ros::Time::now();
#endif

          // We create the edge image from the bufferized events...
          Mat edge_image;
          evts_to_edge_image(evts_to_process, &edge_image, height, width, undistortion_matrix);

          // ... we publish it...
          cv_bridge::CvImage edges_msg;
          edges_msg.encoding = sensor_msgs::image_encodings::MONO8;
          edges_msg.image = edge_image;
          edges_pub.publish(edges_msg.toImageMsg());

#if FPS_MEASUREMENT
          ros::Duration duration = ros::Time::now() - begin;
          myfile << duration.toSec()*1000 << endl;
#endif

          // ... and we sleep in-between two edge images to mimic the behaviour the method would
          // have with a live camera.
          std::chrono::nanoseconds elapsed_time = chrono::system_clock::now() - start_delta_t;

          // If you want however to make the code faster, simply comment the following line :)
          this_thread::sleep_for(chrono::milliseconds(accumul_window) - elapsed_time);

          // Once done, we note the current time as the beginning of the creation of the next edge
          // image
          start_delta_t = chrono::system_clock::now();
        }
      }
    }
  }
}


#if METAVISION_SDK_AVAILABLE
/**
 * \brief Processes all the events from the given .dat file to create the corresponding edge
 * images, which are then sent for further processing.
 *
 * \param dat_path The path to the input .dat file
 * \param calib_path The path to the input calibration file (empty = no calib file)
 * \param accumul_window The time window Δt for accumulating the events (in ms)
 */
void process_events_from_dat(const string& dat_path, const string& calib_path, int accumul_window)
{
  // Opening the .dat file
  ifstream dat_file;
  dat_file.open(dat_path, fstream::binary);
  if(!dat_file.is_open()) {
    ROS_ERROR_STREAM("Could not open .dat file with path '" << dat_path << "', exiting!");
    exit(EXIT_FAILURE);
  }

  // Finding the end of the header
  std::streampos header_end;
  while(true) {
    string header_line;
    getline(dat_file, header_line);
    if(header_line[0] != '%') {
      break;
    }
    header_end = dat_file.tellg();
  }

  // Just in case we hit the end of the stream, we reset the error state
  dat_file.clear();

  // We go back to the end of the header
  // The +2 is because two bytes are used to provide infos before the real data
  // See https://docs.prophesee.ai/stable/data_formats/file_formats/dat.html
  dat_file.seekg((int)header_end+2);

  // Computing the undistortion matrix, used if a calibration file of the camera is given
  Mat undistortion_matrix;
  if(!calib_path.empty()) {
    undistortion_matrix = init_undistort_mat(calib_path, height, width);
  }

  // Extracting a window Δt of events from the dat file might take less time than Δt
  // Therefore, we must track the computation time we take to create each edge image, so that the
  // publish rate of this "replay" module is the same as the one of the "live" module
  chrono::system_clock::time_point start_delta_t = chrono::system_clock::now();

  // Reading the content of the file, and computing the edge images from it
  vector<dvs_msgs::Event> evts_to_process;
  while(dat_file.tellg() != -1) {
    // Reading an event from the file
    Metavision::Event2d::RawEvent evt;
    dat_file.read(reinterpret_cast<char*>(&evt), sizeof(Metavision::Event2d::RawEvent));

    // Converting it to the DVS format for unicity purposes
    dvs_msgs::Event evt_cvt;
    evt_cvt.x = evt.x;
    evt_cvt.y = evt.y;
    evt_cvt.polarity = evt.p;
    evt_cvt.ts = ros::Time().fromSec(evt.ts/1e6);
    evts_to_process.push_back(evt_cvt);

    // We check that the events are in the correct order
    if(evts_to_process.back().ts < evts_to_process.front().ts)
    {
      ROS_WARN("Badly ordered events timestamps, cleaning the queue");
      evts_to_process.clear();
    } else if(evts_to_process.back().ts - evts_to_process.front().ts
      >= ros::Duration().fromSec(accumul_window/1000.))
    {

#if FPS_MEASUREMENT
      ros::Time begin = ros::Time::now();
#endif

      // If this is the case, and if the buffer exceeds the Δt window, we create the edge image
      // from the collected events...
      Mat edge_image;
      evts_to_edge_image(evts_to_process, &edge_image, height, width, undistortion_matrix);

      // ... we publish it...
      cv_bridge::CvImage edges_msg;
      edges_msg.encoding = sensor_msgs::image_encodings::MONO8;
      edges_msg.image = edge_image;
      edges_pub.publish(edges_msg.toImageMsg());

#if FPS_MEASUREMENT
      ros::Duration duration = ros::Time::now() - begin;
      myfile << duration.toSec()*1000 << endl;
#endif

      // ... and do not forget to clear the buffer here, to accumulate new evts
      evts_to_process.clear();

      // Finally, we sleep in-between two edge images to mimic the behaviour the method would have
      // with a live camera.
      std::chrono::nanoseconds elapsed_time = chrono::system_clock::now() - start_delta_t;

      // If you want however to make the code faster, simply comment the following line :)
      this_thread::sleep_for(chrono::milliseconds(accumul_window) - elapsed_time);

      // Once done, we note the current time as the beginning of the creation of the next edge
      // image
      start_delta_t = chrono::system_clock::now();
    }
  }

  // Close the dat file
  dat_file.close();
}
#endif


#if HDF5_SDK_AVAILABLE
/**
 * \brief Processes all the events from the given .h5 file to create the corresponding edge images,
 * which are then sent for further processing.
 *
 * \param h5_path The path to the input .h5 file
 * \param calib_path The path to the input calibration file (empty = no calib file)
 * \param accumul_window The time window Δt for accumulating the events (in ms)
 */
void process_events_from_h5(const string& h5_path, const string& calib_path, int accumul_window)
{
  // Disabling the autoprinting of H5 exceptions
  H5::Exception::dontPrint();

  // Opening the .h5 file
  H5::H5File h5_file;
  try {
    h5_file.openFile(h5_path, H5F_ACC_RDONLY);
  } catch(H5::FileIException&) {
    ROS_ERROR_STREAM("Could not open .h5 file with path '" << h5_path << "', exiting!");
    exit(EXIT_FAILURE);
  }

  // Opening the datasets
  H5::DataSet dataset_x, dataset_y, dataset_t, dataset_p, dataset_t_offset;
  try {
    dataset_x = h5_file.openDataSet("events/x");
    dataset_y = h5_file.openDataSet("events/y");
    dataset_t = h5_file.openDataSet("events/t");
    dataset_p = h5_file.openDataSet("events/p");
    dataset_t_offset = h5_file.openDataSet("/t_offset");
  } catch(H5::FileIException&) {
    ROS_ERROR("Unable to read data from the .h5 file, exiting!");
    exit(EXIT_FAILURE);
  }

  // Extracting the number of events
  hsize_t nb_events;
	dataset_x.getSpace().getSimpleExtentDims(&nb_events);

  // Reading the data and storing it in memory
  uint16_t* evts_x = new uint16_t[nb_events];
  uint16_t* evts_y = new uint16_t[nb_events];
  uint32_t* evts_t = new uint32_t[nb_events];
  uint8_t* evts_p = new uint8_t[nb_events];
  int64_t t_offset;

  dataset_x.read(evts_x, H5::PredType::NATIVE_UINT16);
  dataset_y.read(evts_y, H5::PredType::NATIVE_UINT16);
  dataset_t.read(evts_t, H5::PredType::NATIVE_UINT32);
  dataset_p.read(evts_p, H5::PredType::NATIVE_UINT8);
  dataset_t_offset.read(&t_offset, H5::PredType::NATIVE_INT64);

  // Computing the undistortion matrix, used if a calibration file of the camera is given
  Mat undistortion_matrix;
  if(!calib_path.empty()) {
    undistortion_matrix = init_undistort_mat_h5(calib_path, height, width);
  }

  // Extracting a window Δt of events from the dat file might take less time than Δt
  // Therefore, we must track the computation time we take to create each edge image, so that the
  // publish rate of this "replay" module is the same as the one of the "live" module
  chrono::system_clock::time_point start_delta_t = chrono::system_clock::now();

  // We create an array which will contain the events to process
  vector<dvs_msgs::Event> evts_to_process;

  // For each event extracted from the h5 file...
  for(hsize_t i = 0; i < nb_events; ++i) {
    // We convert it to the DVS format for unicity purposes
    dvs_msgs::Event evt_cvt;
    evt_cvt.x = evts_x[i];
    evt_cvt.y = evts_y[i];
    evt_cvt.polarity = evts_p[i];
    evt_cvt.ts = ros::Time().fromSec((evts_t[i]+t_offset)/1e6);
    evts_to_process.push_back(evt_cvt);

    // We check that the events are in the correct order
    if(evts_to_process.back().ts < evts_to_process.front().ts)
    {
      ROS_WARN("Badly ordered events timestamps, cleaning the queue");
      evts_to_process.clear();
    } else if(evts_to_process.back().ts - evts_to_process.front().ts
      >= ros::Duration().fromSec(accumul_window/1000.))
    {
#if FPS_MEASUREMENT
      ros::Time begin = ros::Time::now();
#endif

      // If this is the case, and if the buffer exceeds the Δt window, we create the edge image
      // from the collected events...
      Mat edge_image;
      evts_to_edge_image(evts_to_process, &edge_image, height, width, undistortion_matrix);

      // ... we publish it...
      cv_bridge::CvImage edges_msg;
      edges_msg.encoding = sensor_msgs::image_encodings::MONO8;
      edges_msg.image = edge_image;
      edges_pub.publish(edges_msg.toImageMsg());

#if FPS_MEASUREMENT
      ros::Duration duration = ros::Time::now() - begin;
      myfile << duration.toSec()*1000 << endl;
#endif

      // ... and do not forget to clear the buffer here, to accumulate new evts
      evts_to_process.clear();

      // Finally, we sleep in-between two edge images to mimic the behaviour the method would have
      // with a live camera.
      std::chrono::nanoseconds elapsed_time = chrono::system_clock::now() - start_delta_t;

      // If you want however to make the code faster, simply comment the following line :)
      this_thread::sleep_for(chrono::milliseconds(accumul_window) - elapsed_time);

      // Once done, we note the current time as the beginning of the creation of the next edge
      // image
      start_delta_t = chrono::system_clock::now();
    }
  }

  // Closing the datasets
  dataset_x.close();
  dataset_y.close();
  dataset_t.close();
  dataset_p.close();
  dataset_t_offset.close();

  // Closing the .h5 file
  h5_file.close();
}
#endif


/**
 * \brief Main function.
 */
int main(int argc, char* argv[])
{
  // ROS node initialization
  ros::init(argc, argv, "edges_node");
  ros::NodeHandle n("~");
  string ns = "/rt_of_low_high_res_event_cameras";

  // Collection of the input file (.bag, .dat, or .h5)
  string input_file;
  n.param<string>("input_file", input_file, "");
  if(input_file.empty()) {
    ROS_ERROR("No input file given, exiting!");
    exit(EXIT_FAILURE);
  }

  // Determining the input format from the filename, to select the correct processing function
  function<void(string, string, int)> file_processing_function;
  string input_format;
  stringstream input_file_s(input_file);
  while(getline(input_file_s, input_format, '.'));
  if(input_format == "bag") {
    file_processing_function = process_events_from_bag;
  } else if(input_format == "dat") {
#if METAVISION_SDK_AVAILABLE
    file_processing_function = process_events_from_dat;
#else
    ROS_ERROR(
      ".dat file used, but support for such files was not enabled! Read the README file to see how\
      to use them. Exiting!");
    exit(EXIT_FAILURE);
#endif
  } else if(input_format == "h5") {
#if HDF5_SDK_AVAILABLE
    file_processing_function = process_events_from_h5;
#else
    ROS_ERROR(
      ".h5 file used, but support for such files was not enabled! Read the README file to see how\
      to use them. Exiting!");
    exit(EXIT_FAILURE);
#endif
  } else {
    ROS_ERROR_STREAM("Unknown input format " << input_format << ", exiting!");
    exit(EXIT_FAILURE);
  }

  // Collection of the input topic, if a rosbag is used
  if(input_format == "bag") {
    n.param<string>("topic", topic, "");
    if(topic.empty()) {
      ROS_ERROR("No input topic selected, exiting!");
      exit(EXIT_FAILURE);
    }
  }

  // Collection of the duration of the accumulation window (Δt, in ms)
  int accumulation_window;
  n.param<int>("accumulation_window", accumulation_window, -1);
  if(accumulation_window <= 0) {
    ROS_ERROR("No accumulation window given, exiting!");
    exit(EXIT_FAILURE);
  }

  // Collection of the image height and width, passed as mandatory parameters
  n.param<int>("height", height, -1);
  n.param<int>("width", width, -1);
  if(height <= 0 || width <= 0) {
    ROS_ERROR("Invalid height/width parameters, exiting!");
    exit(EXIT_FAILURE);
  }

  // Collection of the subscriber/publisher queues size
  int queues_size;
  n.param<int>("queues_size", queues_size, QUEUES_SIZE_DEFAULT);

  // If given, collection of the calibration file
  string calibration_file = n.param<string>("calibration_file", "");

  // Configuration of the edge image publisher
  edges_pub = n.advertise<sensor_msgs::Image>(ns+"/edge_image", queues_size);

#if FPS_MEASUREMENT
  myfile.open(string(FPS_MEASUREMENT_FOLDER_PATH) + "/edges.txt");
#endif

  // Finally, we process all the events from the given file
  file_processing_function(input_file, calibration_file, accumulation_window);

#if FPS_MEASUREMENT
  myfile.close();
#endif

  return 0;
}
