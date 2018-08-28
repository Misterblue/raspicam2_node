#include <RasPiCamPublisherNode.hpp>

#include <class_loader/register_macro.hpp>

RasPiCamPublisher::RasPiCamPublisher() : Node("raspicam2", "", true) {
    init_cam(&state_srv);  // will need to figure out how to handle start and stop with dynamic reconfigure
    pub_img = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed");
    pub_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("image/camera_info");
    srv_info = this->create_service<sensor_msgs::srv::SetCameraInfo>("set_camera_info",
        std::bind(&RasPiCamPublisher::set_camera_info, this,
        std::placeholders::_1, std::placeholders::_2));
    start_capture(&state_srv);
}

RasPiCamPublisher::~RasPiCamPublisher() {
    close_cam(&state_srv);
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
void RasPiCamPublisher::encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    MMAL_BUFFER_HEADER_T *new_buffer;
    int complete = 0;

    // We pass our file handle and other stuff in via the userdata field.

    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
    if (pData && pData->pstate->isInit) {
        int bytes_written = buffer->length;
        if (buffer->length) {
            if (pData->id != INT_MAX) {
                if (pData->id + buffer->length > IMG_BUFFER_SIZE) {
                    pData->id = INT_MAX;  // mark this frame corrupted
                } else {
                    mmal_buffer_header_mem_lock(buffer);
                    memcpy(&(pData->buffer[pData->frame & 1][pData->id]), buffer->data, buffer->length);
                    pData->id += bytes_written;
                    mmal_buffer_header_mem_unlock(buffer);
                }
            }
        }

        if (bytes_written != buffer->length) {
            vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
            pData->abort = 1;
        }

        if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)) {
            complete = 1;
        }

        if (complete) {
            if (pData->id != INT_MAX) {
                if (pData->pThis->skip_frames > 0 && pData->pThis->frames_skipped < pData->pThis->skip_frames) {
                    pData->pThis->frames_skipped++;
                } else {
                    pData->pThis->frames_skipped = 0;
                    sensor_msgs::msg::CompressedImage::UniquePtr msg(new sensor_msgs::msg::CompressedImage());
                    // split timestamp into seconds and nano-seconds
                    const int64_t tnow_ns = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
                    const auto div = std::div(tnow_ns, int64_t(1000000000));
                    msg->header.frame_id = "camera";
                    msg->header.stamp.sec = div.quot;
                    msg->header.stamp.nanosec = div.rem;
                    // set raw compressed data
                    msg->format = "jpeg";
                    msg->data.insert(msg->data.end(), pData->buffer[pData->frame & 1], &(pData->buffer[pData->frame & 1][pData->id]));
                    pData->pThis->pub_img->publish(msg);

                    pData->pThis->camera_info.header.stamp.sec = div.quot;
                    pData->pThis->camera_info.header.stamp.nanosec = div.rem;
                    pData->pThis->pub_info->publish(pData->pThis->camera_info);

                    pData->frame++;
                }
            }
            pData->id = 0;
        }
    }

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

        if (new_buffer) {
            status = mmal_port_send_buffer(port, new_buffer);
        }

        if (!new_buffer || status != MMAL_SUCCESS) {
            vcos_log_error("Unable to return a buffer to the encoder port");
        }
    }
}

/**
* Assign a default set of parameters to the state passed in
*
* @param state Pointer to state structure to assign defaults to
*/
void RasPiCamPublisher::get_status(RASPIVID_STATE *state) {
    if (!state) {
        vcos_assert(0);
        return;
    }

    // get parameters
    int w, h, f, q;
    get_parameter_or("width", w, 1280);
    get_parameter_or("height", h, 960);
    get_parameter_or("fps", f, 2);
    get_parameter_or("quality", q, 80);

    // std::cout << "width: " << w << std::endl;
    // std::cout << "height: " << h << std::endl;
    // std::cout << "fps: " << w << std::endl;
    // std::cout << "quality: " << q << std::endl;

    // set default camera parameters for Camera Module v1
    // https://www.raspberrypi.org/documentation/hardware/camera/
    // camera centre: 3.76 × 2.74 mm
    // focal length: 3.60 mm +/- 0.01
    const double fx = (3.60 / 3.76) * w;
    const double fy = (3.60 / 2.74) * h;
    const double cx = w/2.0;
    const double cy = h/2.0;
    camera_info.width = w;
    camera_info.height = h;
    camera_info.k = {fx, 0,  cx,
                     0,  fy, cy,
                     0,   0, 1};

    // Default everything to zero
    memset(state, 0, sizeof(RASPIVID_STATE));
    state->width = w;
    state->height = h;
    state->quality = q;
    state->framerate = f;
    state->camera_parameters.hflip = 0;
    state->camera_parameters.vflip = 0;
    state->camera_parameters.shutter_speed = 0;

    state->isInit = 0;

    // Setup preview window defaults
    // raspipreview_set_defaults(&state->preview_parameters);

    // Set up the camera_parameters to default
    raspicamcontrol_set_defaults(&state->camera_parameters);
}

/**
* Create the camera component, set up its ports
*
* @param state Pointer to state control struct
*
* @return 0 if failed, pointer to component if successful
*
*/
MMAL_COMPONENT_T *RasPiCamPublisher::create_camera_component(RASPIVID_STATE *state) {
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
    MMAL_STATUS_T status;

    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

    if (status != MMAL_SUCCESS) {
        vcos_log_error("Failed to create camera component");
        goto error;
    }

    if (!camera->output_num) {
        vcos_log_error("Camera doesn't have output ports");
        goto error;
    }

    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    //  set up the camera configuration
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config;
        cam_config.hdr.id = MMAL_PARAMETER_CAMERA_CONFIG;
        cam_config.hdr.size = sizeof(cam_config);
        cam_config.max_stills_w = state->width;
        cam_config.max_stills_h = state->height;
        cam_config.stills_yuv422 = 0;
        cam_config.one_shot_stills = 0;
        cam_config.max_preview_video_w = state->width;
        cam_config.max_preview_video_h = state->height;
        cam_config.num_preview_video_frames = 3;
        cam_config.stills_capture_circular_buffer_height = 0;
        cam_config.fast_preview_resume = 0;
        cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC;

        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }

    // Now set up the port formats

    // Set the encode format on the video  port

    format = video_port->format;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->encoding = MMAL_ENCODING_I420;
    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = state->framerate;
    format->es->video.frame_rate.den = 0;

    status = mmal_port_format_commit(video_port);

    if (status) {
        vcos_log_error("camera video format couldn't be set");
        goto error;
    }

    // Ensure there are enough buffers to avoid dropping frames
    if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

    // Set the encode format on the still  port

    format = still_port->format;

    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = 1;
    format->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(still_port);

    if (status) {
        vcos_log_error("camera still format couldn't be set");
        goto error;
    }

    video_port->buffer_num = video_port->buffer_num_recommended;
    /* Ensure there are enough buffers to avoid dropping frames */
    if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM) {
        still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    }

    /* Enable component */
    status = mmal_component_enable(camera);

    if (status) {
        vcos_log_error("camera component couldn't be enabled");
        goto error;
    }

    raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

    state->camera_component = camera;

    return camera;

error:

    if (camera) {
        mmal_component_destroy(camera);
    }

    return 0;
}

/**
* Destroy the camera component
*
* @param state Pointer to state control struct
*
*/
void RasPiCamPublisher::destroy_camera_component(RASPIVID_STATE *state) {
    if (state->camera_component) {
        mmal_component_destroy(state->camera_component);
        state->camera_component = NULL;
    }
}

/**
* Create the encoder component, set up its ports
*
* @param state Pointer to state control struct
*
* @return MMAL_SUCCESS if all OK, something else otherwise
*
*/
MMAL_STATUS_T RasPiCamPublisher::create_encoder_component(RASPIVID_STATE *state) {
    MMAL_COMPONENT_T *encoder = 0;
    MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
    MMAL_STATUS_T status;
    MMAL_POOL_T *pool;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

    if (status != MMAL_SUCCESS) {
        vcos_log_error("Unable to create video encoder component");
        goto error;
    }

    if (!encoder->input_num || !encoder->output_num) {
        status = MMAL_ENOSYS;
        vcos_log_error("Video encoder doesn't have input/output ports");
        goto error;
    }

    encoder_input = encoder->input[0];
    encoder_output = encoder->output[0];

    // We want same format on input and output
    mmal_format_copy(encoder_output->format, encoder_input->format);

    // Only supporting H264 at the moment
    encoder_output->format->encoding = MMAL_ENCODING_JPEG;

    encoder_output->buffer_size = encoder_output->buffer_size_recommended;

    if (encoder_output->buffer_size < encoder_output->buffer_size_min) {
        encoder_output->buffer_size = encoder_output->buffer_size_min;
    }

    encoder_output->buffer_num = encoder_output->buffer_num_recommended;

    if (encoder_output->buffer_num < encoder_output->buffer_num_min) {
        encoder_output->buffer_num = encoder_output->buffer_num_min;
    }

    // Commit the port changes to the output port
    status = mmal_port_format_commit(encoder_output);

    if (status != MMAL_SUCCESS) {
        vcos_log_error("Unable to set format on video encoder output port");
        goto error;
    }

    // Set the JPEG quality level
    status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state->quality);

    if (status != MMAL_SUCCESS) {
        vcos_log_error("Unable to set JPEG quality");
        goto error;
    }

    //  Enable component
    status = mmal_component_enable(encoder);

    if (status != MMAL_SUCCESS) {
        vcos_log_error("Unable to enable video encoder component");
        goto error;
    }

    /* Create pool of buffer headers for the output port to consume */
    pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

    if (!pool) {
        vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
    }

    state->encoder_pool = pool;
    state->encoder_component = encoder;

    return status;

error:
    if (encoder) {
        mmal_component_destroy(encoder);
    }

    return status;
}

/**
* Destroy the encoder component
*
* @param state Pointer to state control struct
*
*/
void RasPiCamPublisher::destroy_encoder_component(RASPIVID_STATE *state) {
    // Get rid of any port buffers first
    if (state->video_pool) {
        mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
    }

    if (state->encoder_component) {
        mmal_component_destroy(state->encoder_component);
        state->encoder_component = NULL;
    }
}

/**
* Connect two specific ports together
*
* @param output_port Pointer the output port
* @param input_port Pointer the input port
* @param Pointer to a mmal connection pointer, reassigned if function
* successful
* @return Returns a MMAL_STATUS_T giving result of operation
*
*/
MMAL_STATUS_T RasPiCamPublisher::connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection) {
    MMAL_STATUS_T status;

    status = mmal_connection_create( connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

    if (status == MMAL_SUCCESS) {
        status = mmal_connection_enable(*connection);
        if (status != MMAL_SUCCESS) {
            mmal_connection_destroy(*connection);
        }
    }

    return status;
}

/**
* Checks if specified port is valid and enabled, then disables it
*
* @param port  Pointer the port
*
*/
void RasPiCamPublisher::check_disable_port(MMAL_PORT_T *port) {
    if (port && port->is_enabled) {
        mmal_port_disable(port);
    }
}

/**
* init_cam

*/
int RasPiCamPublisher::init_cam(RASPIVID_STATE *state) {
    // Our main data storage vessel..
    MMAL_STATUS_T status;
    MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;
    MMAL_PORT_T *preview_input_port = NULL;
    MMAL_PORT_T *encoder_input_port = NULL;
    MMAL_PORT_T *encoder_output_port = NULL;

    bcm_host_init();
    get_status(state);
    // Register our application with the logging system
    vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

    // OK, we have a nice set of parameters. Now set up our components
    // We have three components. Camera, Preview and encoder.

    if (!create_camera_component(state)) {
    } else if ((status = create_encoder_component(state)) != MMAL_SUCCESS) {
        destroy_camera_component(state);
    } else {
        PORT_USERDATA *callback_data_enc = (PORT_USERDATA *)malloc(sizeof(PORT_USERDATA));
        camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        camera_still_port = state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        encoder_input_port = state->encoder_component->input[0];
        encoder_output_port = state->encoder_component->output[0];
        status = connect_ports(camera_video_port, encoder_input_port, &state->encoder_connection);
        if (status != MMAL_SUCCESS) {
            return 1;
        }
        callback_data_enc->buffer[0] = (unsigned char *)malloc(IMG_BUFFER_SIZE);
        callback_data_enc->buffer[1] = (unsigned char *)malloc(IMG_BUFFER_SIZE);
        // Set up our userdata - this is passed though to the callback where
        // we need the information.
        callback_data_enc->pstate = state;
        callback_data_enc->abort = 0;
        callback_data_enc->id = 0;
        callback_data_enc->frame = 0;
        callback_data_enc->pThis = this;
        encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)callback_data_enc;
        PORT_USERDATA *pData = (PORT_USERDATA *)encoder_output_port->userdata;
        // Enable the encoder output port and tell it its callback function
        status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);
        if (status != MMAL_SUCCESS) {
            return 1;
        }
        state->isInit = 1;
    }
    return 0;
}

int RasPiCamPublisher::start_capture(RASPIVID_STATE *state) {
    if (!(state->isInit)) init_cam(state);
    MMAL_PORT_T *camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
    MMAL_PORT_T *encoder_output_port = state->encoder_component->output[0];

    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        return 1;
    }
    // Send all the buffers to the video port
    {
        int num = mmal_queue_length(state->encoder_pool->queue);
        int q;
        for (q = 0; q < num; q++) {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state->encoder_pool->queue);

            if (!buffer) {
                vcos_log_error("Unable to get a required buffer %d from pool queue", q);
            }

            if (mmal_port_send_buffer(encoder_output_port, buffer) != MMAL_SUCCESS) {
                vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
            }
        }
    }
    return 0;
}

int RasPiCamPublisher::close_cam(RASPIVID_STATE *state) {
    if (state->isInit) {
        state->isInit = 0;
        MMAL_COMPONENT_T *camera = state->camera_component;
        MMAL_COMPONENT_T *encoder = state->encoder_component;
        MMAL_PORT_T *encoder_output_port = state->encoder_component->output[0];
        MMAL_PORT_T *camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];
        PORT_USERDATA *pData = (PORT_USERDATA *)encoder_output_port->userdata;

        if (camera_still_port && camera_still_port->is_enabled) {
            mmal_port_disable(camera_still_port);
        }

        if (encoder->output[0] && encoder->output[0]->is_enabled) {
            mmal_port_disable(encoder->output[0]);
        }

        mmal_connection_destroy(state->encoder_connection);

        // Disable components
        if (encoder) {
            mmal_component_disable(encoder);
        }

        if (camera) {
            mmal_component_disable(camera);
        }

        // Destroy encoder component
        // Get rid of any port buffers first
        if (state->encoder_pool) {
            mmal_port_pool_destroy(encoder->output[0], state->encoder_pool);
        }

        free(pData->buffer[0]);
        free(pData->buffer[1]);

        if (encoder) {
            mmal_component_destroy(encoder);
            encoder = NULL;
        }
        // destroy camera component
        if (camera) {
            mmal_component_destroy(camera);
            camera = NULL;
        }
        return 0;
    } else
        return 1;
}

void RasPiCamPublisher::set_camera_info(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res)
{
    camera_info = req->camera_info;
    res->success = true;
}

CLASS_LOADER_REGISTER_CLASS(RasPiCamPublisher, rclcpp::Node)

