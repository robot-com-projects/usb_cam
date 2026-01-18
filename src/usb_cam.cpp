// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#define CLEAR(x) memset(&(x), 0, sizeof(x))

extern "C" {
#include <linux/videodev2.h>  // Defines V4L2 format constants
#include <malloc.h>  // for memalign
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat
#include <unistd.h>  // for getpagesize()
#include <fcntl.h>  // for O_* constants and open()
}

#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <thread>

#include "opencv2/imgproc.hpp"

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/conversions.hpp"
#include "usb_cam/utils.hpp"


namespace usb_cam
{

using utils::io_method_t;


UsbCam::UsbCam()
: m_device_name(), m_io(io_method_t::IO_METHOD_MMAP), m_fd(-1),
  m_number_of_buffers(4), m_buffers(new usb_cam::utils::buffer[m_number_of_buffers]), m_image(),
  m_avframe(NULL), m_avcodec(NULL), m_avoptions(NULL),
  m_avcodec_context(NULL), m_is_capturing(false), m_framerate(0),
  m_epoch_time_shift_us(usb_cam::utils::get_epoch_time_shift_us()), m_supported_formats(),
  m_enable_undistortion(false), m_camera_info_url()
{}

UsbCam::~UsbCam()
{
  shutdown();
}


/// @brief Fill destination image with source image. If required, convert a given
/// V4L2 Image into another type. Look up possible V4L2 pixe formats in the
/// `linux/videodev2.h` header file.
/// @param src a pointer to a V4L2 source image
/// @param dest a pointer to where the source image should be copied (if required)
/// @param bytes_used number of bytes used by the src buffer
void UsbCam::process_image(const char * src, char * & dest, const int & bytes_used)
{
  // TODO(flynneva): could we skip the copy here somehow?
  // If no conversion required, just copy the image from V4L2 buffer
  if (m_image.pixel_format->requires_conversion() == false) {
    memcpy(dest, src, m_image.size_in_bytes);
  } else {
    m_image.pixel_format->convert(src, dest, bytes_used);
  }
  
  // Apply undistortion if enabled
  if (m_enable_undistortion && !m_map1.empty() && !m_map2.empty()) {
    // Convert the image data to OpenCV Mat
    cv::Mat input_image;
    
    // Determine the OpenCV format based on pixel format
    if (m_image.pixel_format->name() == "mjpeg2rgb" || 
        m_image.pixel_format->name() == "yuyv2rgb" ||
        m_image.pixel_format->name() == "uyvy2rgb") {
      // RGB format
      input_image = cv::Mat(m_image.height, m_image.width, CV_8UC3, dest);
    } else if (m_image.pixel_format->name() == "mono8") {
      // Grayscale format
      input_image = cv::Mat(m_image.height, m_image.width, CV_8UC1, dest);
    } else {
      // Default to RGB for other formats
      input_image = cv::Mat(m_image.height, m_image.width, CV_8UC3, dest);
    }
    
    // Apply fisheye undistortion using pre-computed maps
    cv::remap(input_image, m_undistorted_image, m_map1, m_map2, 
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    
    // Copy the undistorted image back to the destination buffer
    memcpy(dest, m_undistorted_image.data, m_image.size_in_bytes);
  }
}

void UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      len = read(m_fd, m_buffers[0].start, m_buffers[0].length);
      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to read frame");
        }
      }
      return process_image(m_buffers[0].start, m_image.data, len);
    case io_method_t::IO_METHOD_MMAP:
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      m_image.v4l2_fmt.type = buf.type;
      buf.memory = V4L2_MEMORY_MMAP;

      // Get current v4l2 pixel format
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Invalid v4l2 format");
        }
      }
      /// Dequeue buffer with the new image
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to retrieve frame with mmap");
        }
      }

      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);

      assert(buf.index < m_number_of_buffers);
      process_image(m_buffers[buf.index].start, m_image.data, buf.bytesused);

      /// Requeue buffer so it can be reused
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with the driver");
      }
      return;
    case io_method_t::IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to exchange buffer with driver");
        }
      }

      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);

      for (i = 0; i < m_number_of_buffers; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(m_buffers[i].start) && \
          buf.length == m_buffers[i].length)
        {
          return;
        }
      }

      assert(i < m_number_of_buffers);
      process_image(reinterpret_cast<const char *>(buf.m.userptr), m_image.data, buf.bytesused);
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with driver");
      }
      return;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
}

void UsbCam::stop_capturing()
{
  if (!m_is_capturing) {return;}

  m_is_capturing = false;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      return;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMOFF, &type)) {
        // Set capturing variable to true again, since stream was not stopped successfully
        m_is_capturing = true;
        throw std::runtime_error("Unable to stop capturing stream");
      }
      return;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
}

void UsbCam::start_capturing()
{
  if (m_is_capturing) {return;}

  unsigned int i;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      break;
    case io_method_t::IO_METHOD_MMAP:
      // Queue the buffers
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to queue image buffer");
        }
      }

      // Start the stream
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_USERPTR:
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(m_buffers[i].start);
        buf.length = m_buffers[i].length;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to configure stream");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
  m_is_capturing = true;
}

void UsbCam::uninit_device()
{
  m_buffers.reset();
}

void UsbCam::init_read()
{
  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  m_buffers[0].length = m_image.size_in_bytes;

  if (!m_buffers[0].start) {
    throw std::overflow_error("Out of memory");
  }
}

void UsbCam::init_mmap()
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    if (EINVAL == errno) {
      throw std::runtime_error("Device does not support memory mapping");
    } else {
      throw std::runtime_error("Unable to initialize memory mapping");
    }
  }

  if (req.count < m_number_of_buffers) {
    throw std::overflow_error("Insufficient buffer memory on device");
  }

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = current_buffer;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYBUF), &buf)) {
      throw std::runtime_error("Unable to query status of buffer");
    }

    m_buffers[current_buffer].length = buf.length;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(mmap(
        NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
        MAP_SHARED /* recommended */, m_fd, buf.m.offset));

    if (MAP_FAILED == m_buffers[current_buffer].start) {
      throw std::runtime_error("Unable to allocate memory for image buffers");
    }
  }
}

void UsbCam::init_userp()
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  auto buffer_size = (m_image.size_in_bytes + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device does not support user pointer i/o");
    } else {
      throw std::invalid_argument("Unable to initialize memory mapping");
    }
  }

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    m_buffers[current_buffer].length = buffer_size;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(memalign(/* boundary */ page_size, buffer_size));

    if (!m_buffers[current_buffer].start) {
      throw std::overflow_error("Out of memory");
    }
  }
}

void UsbCam::init_device()
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCAP), &cap)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device is not a V4L2 device");
    } else {
      throw std::invalid_argument("Unable to query device capabilities");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    throw std::invalid_argument("Device is not a video capture device");
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        throw std::invalid_argument("Device does not support read i/o");
      }
      break;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        throw std::invalid_argument("Device does not support streaming i/o");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("Unsupported IO method specified");
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_CROPCAP), &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_image.v4l2_fmt.fmt.pix.width = m_image.width;
  m_image.v4l2_fmt.fmt.pix.height = m_image.height;
  m_image.v4l2_fmt.fmt.pix.pixelformat = m_image.pixel_format->v4l2();
  m_image.v4l2_fmt.fmt.pix.field = V4L2_FIELD_ANY;

  // Set v4l2 capture format
  // Note VIDIOC_S_FMT may change width and height
  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_FMT), &m_image.v4l2_fmt)) {
    throw strerror(errno);
  }

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0) {
    throw strerror(errno);
  }

  if (!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME) {
    throw "V4L2_CAP_TIMEPERFRAME not supported";
  }

  // TODO(lucasw) need to get list of valid numerator/denominator pairs
  // and match closest to what user put in.
  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = m_framerate;
  if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0) {
    throw std::invalid_argument("Couldn't set camera framerate");
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      init_read();
      break;
    case io_method_t::IO_METHOD_MMAP:
      init_mmap();
      break;
    case io_method_t::IO_METHOD_USERPTR:
      init_userp();
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something
      break;
  }
}

void UsbCam::close_device()
{
  // Device is already closed
  if (m_fd == -1) {return;}

  if (-1 == close(m_fd)) {
    throw strerror(errno);
  }

  m_fd = -1;
}

void UsbCam::open_device()
{
  struct stat st;

  if (-1 == stat(m_device_name.c_str(), &st)) {
    throw strerror(errno);
  }

  if (!S_ISCHR(st.st_mode)) {
    throw strerror(errno);
  }

  m_fd = open(m_device_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == m_fd) {
    throw strerror(errno);
  }
}

void UsbCam::configure(
  parameters_t & parameters, const io_method_t & io_method)
{
  m_device_name = parameters.device_name;
  m_io = io_method;

  // Open device file descriptor before anything else
  open_device();

  m_image.width = static_cast<int>(parameters.image_width);
  m_image.height = static_cast<int>(parameters.image_height);
  m_image.set_number_of_pixels();

  // Do this before calling set_bytes_per_line and set_size_in_bytes
  m_image.pixel_format = set_pixel_format(parameters);
  m_image.set_bytes_per_line();
  m_image.set_size_in_bytes();
  m_framerate = parameters.framerate;

  // Configure undistortion
  m_enable_undistortion = parameters.enable_undistortion;
  m_camera_info_url = parameters.camera_info_url;
  
  if (m_enable_undistortion) {
    load_camera_calibration();
    init_undistortion_maps();
    // Initialize the undistorted image buffer
    m_undistorted_image = cv::Mat(m_image.height, m_image.width, CV_8UC3);
  }

  init_device();
  
  // Configure camera controls after device initialization (if enabled)
  // This is optional and can be disabled if it causes issues
  if (parameters.enable_camera_controls) {
    try {
      configure_camera_controls(parameters);
    } catch (const std::exception& e) {
      std::cout << "Warning: Camera controls configuration failed: " << e.what() << std::endl;
      std::cout << "Continuing without advanced camera controls..." << std::endl;
    }
  } else {
    std::cout << "Camera controls disabled - using camera defaults" << std::endl;
  }
}

void UsbCam::start()
{
  start_capturing();
  
  // Stabilize auto exposure after starting capture (if enabled)
  // This helps ensure consistent exposure across frames
  if (m_enable_undistortion) {  // Only stabilize if we're doing advanced processing
    stabilize_auto_exposure(5);  // Reduced to 5 frames for safety
  }
}

void UsbCam::shutdown()
{
  stop_capturing();
  uninit_device();
  close_device();
}

/// @brief Grab new image from V4L2 device, return pointer to image
/// @return pointer to image data
char * UsbCam::get_image()
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return nullptr;
  }
  // grab the image
  grab_image();
  return m_image.data;
}

/// @brief Overload get_image so users can pass in an image pointer to fill
/// @param destination destination to fill in with image
void UsbCam::get_image(char * destination)
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return;
  }
  // Set the destination pointer to be filled
  m_image.data = destination;
  // grab the image
  grab_image();
}

std::vector<capture_format_t> UsbCam::get_supported_formats()
{
  m_supported_formats.clear();
  struct v4l2_fmtdesc * current_format = new v4l2_fmtdesc();
  struct v4l2_frmsizeenum * current_size = new v4l2_frmsizeenum();
  struct v4l2_frmivalenum * current_interval = new v4l2_frmivalenum();

  current_format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  current_format->index = 0;
  for (current_format->index = 0;
    usb_cam::utils::xioctl(
      m_fd, VIDIOC_ENUM_FMT, current_format) == 0;
    ++current_format->index)
  {
    current_size->index = 0;
    current_size->pixel_format = current_format->pixelformat;

    for (current_size->index = 0;
      usb_cam::utils::xioctl(
        m_fd, VIDIOC_ENUM_FRAMESIZES, current_size) == 0;
      ++current_size->index)
    {
      current_interval->index = 0;
      current_interval->pixel_format = current_size->pixel_format;
      current_interval->width = current_size->discrete.width;
      current_interval->height = current_size->discrete.height;
      for (current_interval->index = 0;
        usb_cam::utils::xioctl(
          m_fd, VIDIOC_ENUM_FRAMEINTERVALS, current_interval) == 0;
        ++current_interval->index)
      {
        if (current_interval->type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          capture_format_t capture_format;
          capture_format.format = *current_format;
          capture_format.v4l2_fmt = *current_interval;
          m_supported_formats.push_back(capture_format);
        }
      }  // interval loop
    }  // size loop
  }  // fmt loop

  delete (current_format);
  delete (current_size);
  delete (current_interval);

  return m_supported_formats;
}

void UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(m_fd + 1, &fds, NULL, NULL, &tv);

  if (-1 == r) {
    if (EINTR == errno) {
      // interruped (e.g. maybe Ctrl + c) so don't throw anything
      return;
    }

    std::cerr << "Something went wrong, exiting..." << errno << std::endl;
    throw errno;
  }

  if (0 == r) {
    std::cerr << "Select timeout, exiting..." << std::endl;
    throw "select timeout";
  }

  read_frame();
}

// enables/disables auto focus
bool UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCTRL), &queryctrl)) {
    if (errno != EINVAL) {
      std::cerr << "VIDIOC_QUERYCTRL" << std::endl;
      return false;
    } else {
      std::cerr << "V4L2_CID_FOCUS_AUTO is not supported" << std::endl;
      return false;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    std::cerr << "V4L2_CID_FOCUS_AUTO is not supported" << std::endl;
    return false;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_CTRL), &control)) {
      std::cerr << "VIDIOC_S_CTRL" << std::endl;
      return false;
    }
  }
  return true;
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
bool UsbCam::set_v4l_parameter(const std::string & param, int value)
{
  char buf[33];
  snprintf(buf, sizeof(buf), "%i", value);
  return set_v4l_parameter(param, buf);
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
bool UsbCam::set_v4l_parameter(const std::string & param, const std::string & value)
{
  int retcode = 0;
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << m_device_name << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  const int kBufferSize = 256;
  char buffer[kBufferSize];
  FILE * stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream)) {
      if (fgets(buffer, kBufferSize, stream) != NULL) {
        output.append(buffer);
      }
    }
    pclose(stream);
    // any output should be an error
    if (output.length() > 0) {
      std::cout << output.c_str() << std::endl;
      retcode = 1;
    }
  } else {
    std::cerr << "usb_cam_node could not run '" << cmd.c_str() << "'" << std::endl;
    retcode = 1;
  }
  return retcode;
}

void UsbCam::load_camera_calibration()
{
  // Simple YAML parser for camera calibration
  // Extract the file path from package:// URL
  std::string file_path = m_camera_info_url;
  if (file_path.find("package://") == 0) {
    // For now, assume the file is in the usb_cam config directory
    // In a full implementation, you'd resolve the package path properly
    size_t pos = file_path.find("config/");
    if (pos != std::string::npos) {
      file_path = "/workspace/robot/ros2/src/vision/usb_cam/" + file_path.substr(pos);
    }
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Could not open camera calibration file: " << file_path << std::endl;
    m_enable_undistortion = false;
    return;
  }

  std::string line;
  std::vector<double> camera_matrix_data;
  std::vector<double> distortion_data;
  bool reading_camera_matrix = false;
  bool reading_distortion = false;

  while (std::getline(file, line)) {
    // Remove leading/trailing whitespace
    line.erase(0, line.find_first_not_of(" \t"));
    line.erase(line.find_last_not_of(" \t") + 1);

    if (line.find("camera_matrix:") != std::string::npos) {
      reading_camera_matrix = true;
      reading_distortion = false;
      continue;
    } else if (line.find("distortion_coefficients:") != std::string::npos) {
      reading_camera_matrix = false;
      reading_distortion = true;
      continue;
    } else if (line.find("distortion_model:") != std::string::npos ||
               line.find("rectification_matrix:") != std::string::npos ||
               line.find("projection_matrix:") != std::string::npos) {
      reading_camera_matrix = false;
      reading_distortion = false;
      continue;
    }

    if (reading_camera_matrix && line.find("data:") != std::string::npos) {
      // Extract data from line like "  data: [196.81495, 0., 335.01621, ...]"
      size_t start = line.find('[');
      size_t end = line.find(']');
      if (start != std::string::npos && end != std::string::npos) {
        std::string data_str = line.substr(start + 1, end - start - 1);
        std::stringstream ss(data_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
          token.erase(0, token.find_first_not_of(" \t"));
          token.erase(token.find_last_not_of(" \t") + 1);
          if (!token.empty()) {
            camera_matrix_data.push_back(std::stod(token));
          }
        }
      }
    } else if (reading_distortion && line.find("data:") != std::string::npos) {
      // Extract distortion coefficients
      size_t start = line.find('[');
      size_t end = line.find(']');
      if (start != std::string::npos && end != std::string::npos) {
        std::string data_str = line.substr(start + 1, end - start - 1);
        std::stringstream ss(data_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
          token.erase(0, token.find_first_not_of(" \t"));
          token.erase(token.find_last_not_of(" \t") + 1);
          if (!token.empty()) {
            distortion_data.push_back(std::stod(token));
          }
        }
      }
    }
  }

  file.close();

  // Validate and set up camera matrix
  if (camera_matrix_data.size() == 9) {
    m_camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) {
      m_camera_matrix.at<double>(i / 3, i % 3) = camera_matrix_data[i];
    }
    std::cout << "Camera matrix loaded successfully" << std::endl;
  } else {
    std::cerr << "Invalid camera matrix data size: " << camera_matrix_data.size() << std::endl;
    m_enable_undistortion = false;
    return;
  }

  // Validate and set up distortion coefficients
  if (distortion_data.size() >= 4) {
    m_distortion_coeffs = cv::Mat(4, 1, CV_64F);
    for (int i = 0; i < 4; i++) {
      m_distortion_coeffs.at<double>(i, 0) = distortion_data[i];
    }
    std::cout << "Distortion coefficients loaded successfully" << std::endl;
  } else {
    std::cerr << "Invalid distortion coefficients data size: " << distortion_data.size() << std::endl;
    m_enable_undistortion = false;
    return;
  }
}

void UsbCam::init_undistortion_maps()
{
  if (!m_enable_undistortion || m_camera_matrix.empty() || m_distortion_coeffs.empty()) {
    return;
  }

  cv::Size image_size(m_image.width, m_image.height);
  cv::Mat E = cv::Mat::eye(3, 3, CV_64F);

  // Initialize fisheye undistortion maps
  cv::fisheye::initUndistortRectifyMap(
    m_camera_matrix,
    m_distortion_coeffs,
    E,
    m_camera_matrix,
    image_size,
    CV_16SC2,
    m_map1,
    m_map2
  );

  std::cout << "Undistortion maps initialized for fisheye camera" << std::endl;
}

void UsbCam::configure_camera_controls(const parameters_t & parameters)
{
  // Configure camera controls using V4L2 directly (more reliable than v4l2-ctl)
  struct v4l2_control control;
  
  std::cout << "Configuring camera controls..." << std::endl;
  
  // Set auto exposure mode (0.75 = auto, 0.25 = manual in OpenCV terms)
  // In V4L2: V4L2_EXPOSURE_AUTO = 0, V4L2_EXPOSURE_MANUAL = 1, V4L2_EXPOSURE_SHUTTER_PRIORITY = 2, V4L2_EXPOSURE_APERTURE_PRIORITY = 3
  if (parameters.autoexposure) {
    control.id = V4L2_CID_EXPOSURE_AUTO;
    control.value = V4L2_EXPOSURE_AUTO;  // Auto exposure
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Auto exposure enabled" << std::endl;
    } else {
      std::cout << "Warning: Could not enable auto exposure" << std::endl;
    }
  } else {
    control.id = V4L2_CID_EXPOSURE_AUTO;
    control.value = V4L2_EXPOSURE_MANUAL;  // Manual exposure
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Manual exposure enabled" << std::endl;
      
      // Set manual exposure value if specified
      if (parameters.exposure > 0) {
        control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        control.value = parameters.exposure;
        if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
          std::cout << "Manual exposure set to: " << parameters.exposure << std::endl;
        }
      }
    } else {
      std::cout << "Warning: Could not set manual exposure" << std::endl;
    }
  }
  
  // Set auto white balance
  if (parameters.auto_white_balance) {
    control.id = V4L2_CID_AUTO_WHITE_BALANCE;
    control.value = 1;  // Enable auto white balance
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Auto white balance enabled" << std::endl;
    } else {
      std::cout << "Warning: Could not enable auto white balance" << std::endl;
    }
  } else {
    control.id = V4L2_CID_AUTO_WHITE_BALANCE;
    control.value = 0;  // Disable auto white balance
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Auto white balance disabled" << std::endl;
      
      // Set manual white balance if specified
      if (parameters.white_balance > 0) {
        control.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
        control.value = parameters.white_balance;
        if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
          std::cout << "White balance temperature set to: " << parameters.white_balance << std::endl;
        }
      }
    }
  }
  
  // Set conservative camera settings (like your Python script)
  // Reset brightness to default (0 = auto/default)
  if (parameters.brightness >= 0) {
    control.id = V4L2_CID_BRIGHTNESS;
    control.value = parameters.brightness;
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Brightness set to: " << parameters.brightness << std::endl;
    }
  }
  
  // Set moderate contrast (32 is a good default like in your Python script)
  if (parameters.contrast >= 0) {
    control.id = V4L2_CID_CONTRAST;
    control.value = parameters.contrast;
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Contrast set to: " << parameters.contrast << std::endl;
    }
  }
  
  // Set moderate saturation
  if (parameters.saturation >= 0) {
    control.id = V4L2_CID_SATURATION;
    control.value = parameters.saturation;
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Saturation set to: " << parameters.saturation << std::endl;
    }
  }
  
  // Set auto gain (0 = auto)
  if (parameters.gain >= 0) {
    control.id = V4L2_CID_GAIN;
    control.value = parameters.gain;
    if (usb_cam::utils::xioctl(m_fd, VIDIOC_S_CTRL, &control) == 0) {
      std::cout << "Gain set to: " << parameters.gain << std::endl;
    }
  }
  
  std::cout << "Camera controls configuration complete" << std::endl;
}

void UsbCam::stabilize_auto_exposure(int frames)
{
  if (!m_is_capturing || frames <= 0) {
    return;
  }
  
  std::cout << "Stabilizing auto exposure (" << frames << " frames)..." << std::endl;
  
  // Use a safer approach - just wait for the camera to stabilize
  // instead of actively reading frames which can cause crashes
  std::this_thread::sleep_for(std::chrono::milliseconds(frames * 100));
  
  std::cout << "Auto exposure stabilization complete" << std::endl;
}

}  // namespace usb_cam
